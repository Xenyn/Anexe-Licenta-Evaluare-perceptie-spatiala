%% Analiza increderii camerei in functie de vizibilitatea in ceata
% Rulati generic_uservehicle, apoi acest script. Modelul furnizeaza:
%   incredereCamera          - iesirea Confidence a senzorului camera, in [0, 1]
%   razaVizibilitateCeata_m  - Env.VisRangeInFog din CarMaker, in metri
%
% Observatie metodologica:
%   Acest script evalueaza factorul de incredere furnizat de camera
%   prin semnalul brut CarMaker numit Confidence.
%   Disponibilitatea raportata aici inseamna procentul de esantioane pentru
%   care factorul de incredere este nenul dupa eliminarea datelor invalide. Validarea
%   completa a obiectului, pe baza ObjID si nVisPixels, ramane in modelul
%   Simulink si nu este recalculata in acest script.

close all;
clc;

latimeIntervalCeata_m = 5;
numarMinimEsantioaneInterval = 20;
directorIesire = fileparts(mfilename("fullpath"));
fisierRezultate = fullfile(directorIesire,"rezultate_incredere_camera_dupa_vizibilitate.csv");

%% Citirea semnalelor salvate de Simulink
if ~exist("incredereCamera","var")
    error("incredereCamera lipseste. Rulati generic_uservehicle mai intai.");
end
if ~exist("razaVizibilitateCeata_m","var")
    error("razaVizibilitateCeata_m lipseste. Rulati generic_uservehicle mai intai.");
end

[timpIncredere_s,incredere] = extrageSemnalInregistrat(incredereCamera);
[timpCeata_s,razaCeata_m] = extrageSemnalInregistrat(razaVizibilitateCeata_m);

timpIncredere_s = timpIncredere_s(:);
incredere = incredere(:);
timpCeata_s = timpCeata_s(:);
razaCeata_m = razaCeata_m(:);

%% Sincronizarea razei de vizibilitate cu esantioanele factorului de incredere
if isequal(timpIncredere_s,timpCeata_s)
    razaCeataSincronizata_m = razaCeata_m;
else
    [timpCeata_s,indiceUnic] = unique(timpCeata_s,"stable");
    razaCeata_m = razaCeata_m(indiceUnic);
    razaCeataSincronizata_m = interp1(timpCeata_s,razaCeata_m,timpIncredere_s, ...
        "previous","extrap");
end

%% Filtrarea esantioanelor invalide
esantionCeataNegativa = isfinite(razaCeataSincronizata_m) & razaCeataSincronizata_m < 0;
if any(esantionCeataNegativa)
    warning("Au fost excluse %d esantioane cu raza de vizibilitate negativa.", ...
        nnz(esantionCeataNegativa));
end

esantionValid = isfinite(incredere) & isfinite(razaCeataSincronizata_m) & ...
    razaCeataSincronizata_m >= 0;
timpIncredere_s = timpIncredere_s(esantionValid);
incredere = incredere(esantionValid);
razaCeataSincronizata_m = razaCeataSincronizata_m(esantionValid);

if isempty(incredere)
    error("Semnalele nu contin esantioane finite si suprapuse in timp.");
end

if any(incredere < 0 | incredere > 1)
    warning("Factorul de încredere contine valori in afara intervalului [0, 1]. " + ...
        "Valorile sunt limitate numai pentru reprezentarea procentuala.");
end

%% Calculul indicatorilor pe esantioane
incredereLimitata = min(max(incredere,0),1);
masuratori = table(timpIncredere_s,razaCeataSincronizata_m, ...
    incredereLimitata .* 100,incredereLimitata > 0, ...
    VariableNames=["Timp_s" "RazaVizibilitateCeata_m" ...
    "FactorIncredere_pct" "AreFactorIncredereNenul"]);

%% Gruparea esantioanelor pe intervale de vizibilitate
limitaMinima_m = floor(min(masuratori.RazaVizibilitateCeata_m) / latimeIntervalCeata_m) ...
    * latimeIntervalCeata_m;
limitaMaxima_m = ceil(max(masuratori.RazaVizibilitateCeata_m) / latimeIntervalCeata_m) ...
    * latimeIntervalCeata_m;

if limitaMinima_m == limitaMaxima_m
    limitaMinima_m = limitaMinima_m - latimeIntervalCeata_m / 2;
    limitaMaxima_m = limitaMaxima_m + latimeIntervalCeata_m / 2;
end

limiteCeata_m = limitaMinima_m:latimeIntervalCeata_m:(limitaMaxima_m + latimeIntervalCeata_m);
masuratori.IntervalCeata = discretize(masuratori.RazaVizibilitateCeata_m,limiteCeata_m);
masuratori = masuratori(~ismissing(masuratori.IntervalCeata),:);

indicatoriIncredere = groupsummary(masuratori,"IntervalCeata", ...
    ["mean" "median" "std"],"FactorIncredere_pct");
indicatoriCeata = groupsummary(masuratori,"IntervalCeata","mean","RazaVizibilitateCeata_m");
indicatoriDisponibilitate = groupsummary(masuratori,"IntervalCeata","mean","AreFactorIncredereNenul");

indicatori = table( ...
    indicatoriCeata.mean_RazaVizibilitateCeata_m, ...
    indicatoriIncredere.GroupCount, ...
    indicatoriIncredere.mean_FactorIncredere_pct, ...
    indicatoriIncredere.median_FactorIncredere_pct, ...
    indicatoriIncredere.std_FactorIncredere_pct, ...
    indicatoriDisponibilitate.mean_AreFactorIncredereNenul .* 100, ...
    VariableNames=["RazaVizibilitateCeata_m" "NumarEsantioane" ...
    "FactorIncredereMediu_pct" "FactorIncredereMedian_pct" ...
    "AbatereStandardFactorIncredere_pct" "DisponibilitateFactorIncredere_pct"]);
indicatori = sortrows(indicatori,"RazaVizibilitateCeata_m");
indicatori = indicatori(indicatori.NumarEsantioane >= numarMinimEsantioaneInterval,:);

if isempty(indicatori)
    error("Niciun interval de vizibilitate nu contine cel putin %d esantioane.", ...
        numarMinimEsantioaneInterval);
end

writetable(indicatori,fisierRezultate);
disp(indicatori);

%% Reprezentarea grafica
limitaInferioara = max(indicatori.FactorIncredereMediu_pct - ...
    indicatori.AbatereStandardFactorIncredere_pct,0);
limitaSuperioara = min(indicatori.FactorIncredereMediu_pct + ...
    indicatori.AbatereStandardFactorIncredere_pct,100);

figure(Name="Factorul de încredere al camerei in functie de ceață", ...
    Color="w",Position=[100 80 1400 850]);
axaIncredere = axes;
title(axaIncredere,"Factorul de încredere al camerei in funcție de vizibilitatea in ceață", ...
    FontSize=17,FontWeight="bold",Color=[0.1 0.1 0.1]);

scatter(masuratori.RazaVizibilitateCeata_m,masuratori.FactorIncredere_pct,10, ...
    [0.15 0.15 0.15],"filled",MarkerFaceAlpha=0.20, ...
    MarkerEdgeAlpha=0.20,DisplayName="Factor de încredere instantaneu");
hold(axaIncredere,"on");
fill([indicatori.RazaVizibilitateCeata_m; flipud(indicatori.RazaVizibilitateCeata_m)], ...
    [limitaInferioara; flipud(limitaSuperioara)],[0.00 0.75 1.00], ...
    FaceAlpha=0.18,EdgeColor="none",DisplayName="Media +/- abaterea standard");
plot(indicatori.RazaVizibilitateCeata_m,indicatori.FactorIncredereMediu_pct,"o-", ...
    LineWidth=2.8,MarkerSize=7,Color=[0.00 0.20 1.00], ...
    MarkerFaceColor="w",DisplayName="Factor de încredere mediu");
plot(indicatori.RazaVizibilitateCeata_m,indicatori.FactorIncredereMedian_pct,"s--", ...
    LineWidth=2.4,MarkerSize=6,Color=[1.00 0.20 0.00], ...
    MarkerFaceColor="w",DisplayName="Factor de încredere median");
stilizeazaAxa(axaIncredere);
ylim(axaIncredere,[0 100]);
xlabel(axaIncredere,"Raza de vizibilitate în ceață (m)",FontSize=13);
ylabel(axaIncredere,"Factor de încredere cameră (%)",FontSize=13);
legend(axaIncredere,Location="east",Orientation="vertical",Color="w", ...
    TextColor=[0.1 0.1 0.1],EdgeColor=[0.6 0.6 0.6],FontSize=13);

%% Functii locale
function [timp_s,date] = extrageSemnalInregistrat(semnalInregistrat)
if isa(semnalInregistrat,"timeseries")
    timp_s = semnalInregistrat.Time;
    date = semnalInregistrat.Data;
elseif isstruct(semnalInregistrat) && isfield(semnalInregistrat,"time") && ...
        isfield(semnalInregistrat,"signals")
    timp_s = semnalInregistrat.time;
    date = semnalInregistrat.signals.values;
else
    error("Format nesuportat. Folosiți Timeseries sau Structure With Time.");
end

date = squeeze(date);
if ~isvector(date)
    error("Fiecare semnal trebuie sa contina o valoare scalara per esantion.");
end
end

function stilizeazaAxa(manerAxa)
set(manerAxa,Color="w",XColor=[0.1 0.1 0.1],YColor=[0.1 0.1 0.1], ...
    FontSize=12,FontName="Arial",LineWidth=1,GridColor=[0.75 0.75 0.75], ...
    MinorGridColor=[0.88 0.88 0.88],GridAlpha=0.8,MinorGridAlpha=0.7);
grid(manerAxa,"on");
grid(manerAxa,"minor");
end
