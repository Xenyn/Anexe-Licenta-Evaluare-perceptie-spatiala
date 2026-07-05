%% Eroarea distantei camerei in functie de vizibilitatea in ceata
% Rulati generic_uservehicle, apoi acest script. Modelul furnizeaza:
%   eroareDistantaCamera_m    - MBR_BL_X minus distanta reala de referinta
%   razaVizibilitateCeata_m   - Env.VisRangeInFog din CarMaker, in metri
%
% Metoda folosita:
%   1. sincronizeaza eroarea camerei cu raza de vizibilitate in ceata;
%   2. elimina esantioanele invalide sau cu vizibilitate negativa;
%   3. elimina erorile procentuale aflate in afara intervalului +/-20%;
%   4. grupeaza rezultatele pe intervale de 5 m ale vizibilitatii;
%   5. calculeaza eroarea medie, MAE si RMSE.

close all;
clc;

distantaReferinta_m = 200;
latimeIntervalCeata_m = 5;
eroareAbsolutaMaxima_pct = 30;
directorIesire = fileparts(mfilename("fullpath"));
fisierRezultate = fullfile(directorIesire,"rezultate_eroare_distanta_dupa_vizibilitate.csv");

%% Citirea semnalelor salvate de Simulink
if ~exist("eroareDistantaCamera_m","var")
    error("eroareDistantaCamera_m lipseste. Rulati generic_uservehicle mai intai.");
end
if ~exist("razaVizibilitateCeata_m","var")
    error("razaVizibilitateCeata_m lipseste. Rulati generic_uservehicle mai intai.");
end

[timpEroare_s,eroareDistanta_m] = extrageSemnalInregistrat(eroareDistantaCamera_m);
[timpCeata_s,razaCeata_m] = extrageSemnalInregistrat(razaVizibilitateCeata_m);

timpEroare_s = timpEroare_s(:);
eroareDistanta_m = eroareDistanta_m(:);
timpCeata_s = timpCeata_s(:);
razaCeata_m = razaCeata_m(:);

%% Sincronizarea razei de vizibilitate cu esantioanele erorii
if isequal(timpEroare_s,timpCeata_s)
    razaCeataSincronizata_m = razaCeata_m;
else
    [timpCeata_s,indiceUnic] = unique(timpCeata_s,"stable");
    razaCeata_m = razaCeata_m(indiceUnic);
    razaCeataSincronizata_m = interp1(timpCeata_s,razaCeata_m,timpEroare_s, ...
        "previous","extrap");
end

%% Filtrarea esantioanelor invalide
esantionCeataNegativa = isfinite(razaCeataSincronizata_m) & razaCeataSincronizata_m < 0;
if any(esantionCeataNegativa)
    warning("Au fost excluse %d esantioane cu raza de vizibilitate negativa.", ...
        nnz(esantionCeataNegativa));
end

esantionValid = isfinite(eroareDistanta_m) & isfinite(razaCeataSincronizata_m) & ...
    razaCeataSincronizata_m >= 0;
timpEroare_s = timpEroare_s(esantionValid);
eroareDistanta_m = eroareDistanta_m(esantionValid);
razaCeataSincronizata_m = razaCeataSincronizata_m(esantionValid);

eroareDistanta_pct = eroareDistanta_m ./ distantaReferinta_m .* 100;
esantionEroareExcesiva = abs(eroareDistanta_pct) > eroareAbsolutaMaxima_pct;
if any(esantionEroareExcesiva)
    warning("Au fost excluse %d esantioane cu eroare absoluta peste %.1f%%.", ...
        nnz(esantionEroareExcesiva),eroareAbsolutaMaxima_pct);
end

esantionRetinut = ~esantionEroareExcesiva;
timpEroare_s = timpEroare_s(esantionRetinut);
eroareDistanta_m = eroareDistanta_m(esantionRetinut);
eroareDistanta_pct = eroareDistanta_pct(esantionRetinut);
razaCeataSincronizata_m = razaCeataSincronizata_m(esantionRetinut);

if isempty(eroareDistanta_m)
    error("Nu au ramas esantioane dupa filtrarea erorilor la +/- %.1f%%.", ...
        eroareAbsolutaMaxima_pct);
end

%% Calculul indicatorilor pe esantioane
masuratori = table(timpEroare_s,razaCeataSincronizata_m,eroareDistanta_m, ...
    eroareDistanta_pct,VariableNames=["Timp_s" "RazaVizibilitateCeata_m" ...
    "EroareDistanta_m" "EroareDistanta_pct"]);
masuratori.EroareAbsoluta_m = abs(masuratori.EroareDistanta_m);
masuratori.EroarePatratica_m2 = masuratori.EroareDistanta_m.^2;

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

indicatori = groupsummary(masuratori,"IntervalCeata","mean", ...
    ["RazaVizibilitateCeata_m" "EroareDistanta_pct" ...
    "EroareAbsoluta_m" "EroarePatratica_m2"]);
indicatori = renamevars(indicatori, ...
    ["GroupCount" "mean_RazaVizibilitateCeata_m" "mean_EroareDistanta_pct" ...
    "mean_EroareAbsoluta_m"], ...
    ["NumarEsantioane" "RazaVizibilitateCeata_m" "EroareMedie_pct" "MAE_m"]);
indicatori.RMSE_m = sqrt(indicatori.mean_EroarePatratica_m2);
indicatori.MAE_pct = indicatori.MAE_m ./ distantaReferinta_m .* 100;
indicatori.RMSE_pct = indicatori.RMSE_m ./ distantaReferinta_m .* 100;
indicatori = removevars(indicatori,["IntervalCeata" "mean_EroarePatratica_m2"]);
indicatori = sortrows(indicatori,"RazaVizibilitateCeata_m");
indicatori = indicatori(:,["RazaVizibilitateCeata_m" "NumarEsantioane" ...
    "EroareMedie_pct" "MAE_m" "MAE_pct" "RMSE_m" "RMSE_pct"]);

writetable(indicatori,fisierRezultate);
disp(indicatori);

%% Reprezentarea grafica
textReferinta = replace(compose("%.1f",distantaReferinta_m),".",",");

figure(Name="Eroarea procentuala a distanței în funcție de ceață", ...
    Color="w",Position=[100 80 1400 850]);
aranjament = tiledlayout(2,1,TileSpacing="compact",Padding="compact");
title(aranjament,"Eroarea procentuală a distanței în funcție de vizibilitatea în ceață" ...
    + newline + "Distanța reală de referință: " + textReferinta + " m", ...
    FontSize=17,FontWeight="bold",Color=[0.1 0.1 0.1]);

axaEroare = nexttile;
scatter(masuratori.RazaVizibilitateCeata_m,masuratori.EroareDistanta_pct,12, ...
    [0.45 0.45 0.45],"filled",MarkerFaceAlpha=0.16, ...
    MarkerEdgeAlpha=0.16,DisplayName="Eroare procentuală");
hold(axaEroare,"on");
plot(indicatori.RazaVizibilitateCeata_m,indicatori.EroareMedie_pct,"o-", ...
    LineWidth=2.2,MarkerSize=6,Color=[0.00 0.45 0.74], ...
    MarkerFaceColor="w",DisplayName=sprintf("Eroare medie (%g m)",latimeIntervalCeata_m));
yline(axaEroare,0,"--",Color=[0.2 0.2 0.2],HandleVisibility="off");
stilizeazaAxa(axaEroare);
ylim(axaEroare,[-eroareAbsolutaMaxima_pct eroareAbsolutaMaxima_pct]);
xlabel(axaEroare,"Raza de vizibilitate în ceață (m)",FontSize=13);
ylabel(axaEroare,"Eroare față de distanța reală (%)",FontSize=13);
legend(axaEroare,Location="north",Orientation="horizontal",Color="w", ...
    TextColor=[0.1 0.1 0.1],EdgeColor=[0.6 0.6 0.6],FontSize=11);

axaIndicatori = nexttile;
plot(indicatori.RazaVizibilitateCeata_m,indicatori.MAE_pct,"o-", ...
    LineWidth=2.2,MarkerSize=6,Color=[0.00 0.45 0.74], ...
    MarkerFaceColor="w",DisplayName="MAE");
hold(axaIndicatori,"on");
plot(indicatori.RazaVizibilitateCeata_m,indicatori.RMSE_pct,"s-", ...
    LineWidth=2.2,MarkerSize=6,Color=[0.85 0.33 0.10], ...
    MarkerFaceColor="w",DisplayName="RMSE");
stilizeazaAxa(axaIndicatori);
ylim(axaIndicatori,[0 20]);
xlabel(axaIndicatori,"Raza de vizibilitate în ceață (m)",FontSize=13);
ylabel(axaIndicatori,"MAE și RMSE (%)",FontSize=13);
legend(axaIndicatori,Location="best",Color="w",TextColor=[0.1 0.1 0.1], ...
    EdgeColor=[0.6 0.6 0.6]);

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
    error("Format de semnal nesuportat. Folositi Timeseries sau Structure With Time.");
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
