%% Evaluarea senzorului Radar RSI pe cadre unice
% Rulați TestS cu modelul generic_uservehicle, apoi executați acest script.
% Sursa autoritară este variabila radar_cadru_evaluare produsă de model.

close all;
clc;

if ~exist("folosesteReferintaFixa","var")
    folosesteReferintaFixa = false;
end
if ~exist("referintaFixa_m","var")
    referintaFixa_m = 48;
end
directorIesire = fileparts(mfilename("fullpath"));
fisierCadre = fullfile(directorIesire,"cadre_evaluare_radar_rsi.csv");
fisierRezumat = fullfile(directorIesire,"rezumat_evaluare_radar_rsi.csv");

if ~exist("radar_cadru_evaluare","var")
    error("Variabila radar_cadru_evaluare lipsește. Rulați TestS cu " + ...
        "generic_uservehicle înainte de analiză.");
end

[timpSimulare_s,dateCadre] = extrageMatriceSemnal(radar_cadru_evaluare);
if size(dateCadre,2)~=30
    error("radar_cadru_evaluare trebuie să conțină 30 de coloane.");
end

% Păstrăm exclusiv actualizările marcate drept cadre radar noi.
esteCadruNou = dateCadre(:,11)>0.5 & isfinite(dateCadre(:,1));
dateCadre = dateCadre(esteCadruNou,:);
timpSimulare_s = timpSimulare_s(esteCadruNou);

if isempty(dateCadre)
    error("Nu există cadre radar unice pentru evaluare.");
end

% Protecție suplimentară împotriva cadrelor repetate cu același TimeFired.
[~,indiciUnici] = unique(dateCadre(:,1),"stable");
dateCadre = dateCadre(indiciUnici,:);
timpSimulare_s = timpSimulare_s(indiciUnici);

numeColoane = ["TimpRadar_s","NumarCadru","IndiceSenzor", ...
    "NumarDetectiiBrute","NumarDetectiiValide","NumarClustere", ...
    "ClusterAsociat","FalsePozitive","TintaPrezenta", ...
    "AchizitieValida","CadruNou","AdevaratPozitiv","FalseNegative", ...
    "DistantaGT_m","DistantaRadar_m","EroareDistanta_m", ...
    "EroareAbsoluta_m","EroareDistanta_pct","AzimutGT_deg", ...
    "AzimutRadar_deg","EroareAzimut_deg","VitezaRadialaGT_mps", ...
    "VitezaRadialaRadar_mps","EroareViteza_mps","PutereMedie_dB", ...
    "LatimeCluster_m","PozitieX_m","PozitieY_m", ...
    "PuncteClusterTinta","CodStare"];

cadre = array2table(dateCadre,VariableNames=numeColoane);
cadre = addvars(cadre,timpSimulare_s,Before=1, ...
    NewVariableNames="TimpSimulare_s");

campuriLogice = ["TintaPrezenta","AchizitieValida","CadruNou", ...
    "AdevaratPozitiv","FalseNegative"];
for camp = campuriLogice
    cadre.(camp) = logical(cadre.(camp));
end

if folosesteReferintaFixa
    lipsaGT = ~isfinite(cadre.DistantaGT_m) | cadre.DistantaGT_m<=0;
    cadre.DistantaGT_m(lipsaGT) = referintaFixa_m;
    cadre.EroareDistanta_m(lipsaGT) = ...
        cadre.DistantaRadar_m(lipsaGT)-referintaFixa_m;
    cadre.EroareAbsoluta_m(lipsaGT) = ...
        abs(cadre.EroareDistanta_m(lipsaGT));
    cadre.EroareDistanta_pct(lipsaGT) = ...
        100*cadre.EroareDistanta_m(lipsaGT)/referintaFixa_m;
end

cadreEvaluabile = cadre.AchizitieValida & cadre.TintaPrezenta;
masurariValide = cadre.AdevaratPozitiv & ...
    isfinite(cadre.DistantaRadar_m) & isfinite(cadre.DistantaGT_m);

numarTP = nnz(cadre.AdevaratPozitiv);
numarFP = sum(cadre.FalsePozitive,"omitnan");
numarFN = nnz(cadre.FalseNegative);
precizie_pct = procentSigur(numarTP,numarTP+numarFP);
recall_pct = procentSigur(numarTP,numarTP+numarFN);
f1_pct = medieF1(precizie_pct,recall_pct);
disponibilitate_pct = procentSigur(nnz(masurariValide),nnz(cadreEvaluabile));

eroriDistanta_m = cadre.EroareDistanta_m(masurariValide);
eroriDistanta_pct = cadre.EroareDistanta_pct(masurariValide);
eroriAzimut_deg = cadre.EroareAzimut_deg(masurariValide & ...
    isfinite(cadre.EroareAzimut_deg));
eroriViteza_mps = cadre.EroareViteza_mps(masurariValide & ...
    isfinite(cadre.EroareViteza_mps));

rezumat = table(height(cadre),nnz(cadreEvaluabile),numarTP,numarFP, ...
    numarFN,disponibilitate_pct,precizie_pct,recall_pct,f1_pct, ...
    medieSigura(eroriDistanta_m),maeSigur(eroriDistanta_m), ...
    rmseSigur(eroriDistanta_m),maeSigur(eroriDistanta_pct), ...
    rmseSigur(eroriDistanta_pct),stdSigur(cadre.DistantaRadar_m(masurariValide)), ...
    medieSigura(eroriAzimut_deg),maeSigur(eroriAzimut_deg), ...
    rmseSigur(eroriAzimut_deg),stdSigur(cadre.AzimutRadar_deg(masurariValide)), ...
    medieSigura(eroriViteza_mps),maeSigur(eroriViteza_mps), ...
    rmseSigur(eroriViteza_mps), ...
    stdSigur(cadre.VitezaRadialaRadar_mps(masurariValide)), ...
    medieSigura(cadre.NumarDetectiiBrute), ...
    medieSigura(cadre.NumarDetectiiValide), ...
    medieSigura(cadre.NumarClustere), ...
    VariableNames=["NumarCadre","CadreEvaluabile","AdevaratPozitive", ...
    "FalsePozitive","FalseNegative","DisponibilitateDetectie_pct", ...
    "Precizie_pct","Recall_pct","ScorF1_pct","BiasDistanta_m", ...
    "MAE_Distanta_m","RMSE_Distanta_m","MAE_Distanta_pct", ...
    "RMSE_Distanta_pct","JitterDistanta_m","BiasAzimut_deg", ...
    "MAE_Azimut_deg","RMSE_Azimut_deg","JitterAzimut_deg", ...
    "BiasViteza_mps","MAE_Viteza_mps","RMSE_Viteza_mps", ...
    "JitterViteza_mps","DetectiiBruteMedii","DetectiiValideMedii", ...
    "ClustereMedii"]);

writetable(cadre,fisierCadre);
writetable(rezumat,fisierRezumat);
disp(rezumat);

%% Pregătire statistici pe intervale de distanță
distantePentruBinuri = cadre.DistantaGT_m;
mascaBinuri = cadreEvaluabile & isfinite(distantePentruBinuri) & ...
    distantePentruBinuri>0;
if ~any(mascaBinuri)
    error("Nu există distanțe ground truth valide pentru graficele în funcție de distanță.");
end

pasBin_m = 10;
distantaMin_m = floor(min(distantePentruBinuri(mascaBinuri))/pasBin_m)*pasBin_m;
distantaMax_m = ceil(max(distantePentruBinuri(mascaBinuri))/pasBin_m)*pasBin_m;
if distantaMax_m<=distantaMin_m
    distantaMax_m = distantaMin_m + pasBin_m;
end

marginiDistanta_m = distantaMin_m:pasBin_m:distantaMax_m;
if numel(marginiDistanta_m)<2
    marginiDistanta_m = [distantaMin_m distantaMin_m+pasBin_m];
end
centreDistanta_m = marginiDistanta_m(1:end-1) + diff(marginiDistanta_m)/2;
numarIntervale = numel(centreDistanta_m);

tpDistanta = zeros(numarIntervale,1);
fpDistanta = zeros(numarIntervale,1);
fnDistanta = zeros(numarIntervale,1);
evaluabileDistanta = zeros(numarIntervale,1);
disponibilitateDistanta_pct = nan(numarIntervale,1);
precizieDistanta_pct = nan(numarIntervale,1);
recallDistanta_pct = nan(numarIntervale,1);
f1Distanta_pct = nan(numarIntervale,1);

for k = 1:numarIntervale
    inInterval = mascaBinuri & distantePentruBinuri>=marginiDistanta_m(k) & ...
        distantePentruBinuri<marginiDistanta_m(k+1);
    if k==numarIntervale
        inInterval = mascaBinuri & distantePentruBinuri>=marginiDistanta_m(k) & ...
            distantePentruBinuri<=marginiDistanta_m(k+1);
    end

    evaluabileDistanta(k) = nnz(inInterval);
    tpDistanta(k) = nnz(cadre.AdevaratPozitiv & inInterval);
    fpDistanta(k) = sum(cadre.FalsePozitive(inInterval),"omitnan");
    fnDistanta(k) = nnz(cadre.FalseNegative & inInterval);

    disponibilitateDistanta_pct(k) = procentSigur(tpDistanta(k),evaluabileDistanta(k));
    precizieDistanta_pct(k) = procentSigur(tpDistanta(k),tpDistanta(k)+fpDistanta(k));
    recallDistanta_pct(k) = procentSigur(tpDistanta(k),tpDistanta(k)+fnDistanta(k));
    f1Distanta_pct(k) = medieF1(precizieDistanta_pct(k),recallDistanta_pct(k));
end

tabelCalitateDistanta = table(centreDistanta_m(:),marginiDistanta_m(1:end-1).', ...
    marginiDistanta_m(2:end).',evaluabileDistanta,tpDistanta,fpDistanta,fnDistanta, ...
    disponibilitateDistanta_pct,precizieDistanta_pct,recallDistanta_pct,f1Distanta_pct, ...
    VariableNames=["DistantaCentru_m","DistantaMin_m","DistantaMax_m", ...
    "CadreEvaluabile","AdevaratPozitive","FalsePozitive","FalseNegative", ...
    "Disponibilitate_pct","Precizie_pct","Recall_pct","F1_pct"]);
fisierCalitateDistanta = fullfile(directorIesire,"calitate_radar_vs_distanta.csv");
writetable(tabelCalitateDistanta,fisierCalitateDistanta);

%% Figura 1: evaluarea distanței
figuraDistanta = figure(Name="Evaluarea distanței Radar RSI",Color="w", ...
    Position=[80 40 1200 900],InvertHardcopy="off");
aranjamentDistanta = tiledlayout(figuraDistanta,2,1, ...
    TileSpacing="loose",Padding="loose");
title(aranjamentDistanta,"Evaluarea distanței măsurate de Radar RSI", ...
    FontName="Times New Roman",FontSize=20,FontWeight="bold", ...
    Color=[0.08 0.08 0.08]);

axaDistanta = nexttile(aranjamentDistanta);
plot(axaDistanta,cadre.TimpRadar_s,cadre.DistantaGT_m,"--", ...
    LineWidth=1.8,Color=[0.2 0.2 0.2],DisplayName="Ground truth");
hold(axaDistanta,"on");
plot(axaDistanta,cadre.TimpRadar_s,cadre.DistantaRadar_m, ...
    LineWidth=1.5,Color=[0 0.45 0.74],DisplayName="Radar RSI");
marcheazaNedetectii(axaDistanta,cadre);
xlabel(axaDistanta,"Timp (s)");
ylabel(axaDistanta,"Distanță (m)");
title(axaDistanta,"Distanța longitudinală");
legendaDistanta = legend(axaDistanta,Location="best");
stilizeazaLegenda(legendaDistanta);
stilizeazaAxa(axaDistanta);

axaEroare = nexttile(aranjamentDistanta);
plot(axaEroare,cadre.DistantaGT_m,cadre.EroareDistanta_m, ...
    LineStyle="none",Marker=".",MarkerSize=9,Color=[0.85 0.33 0.10], ...
    DisplayName="Eroare radar");
hold(axaEroare,"on");
yline(axaEroare,0,"--",Color=[0.35 0.35 0.35],DisplayName="Referință 0 m");
xlabel(axaEroare,"Distanță ground truth (m)");
ylabel(axaEroare,"Eroare de distanță (m)");
title(axaEroare,sprintf("Eroarea distanței: MAE %.3f m | RMSE %.3f m", ...
    rezumat.MAE_Distanta_m,rezumat.RMSE_Distanta_m));
legendaEroare = legend(axaEroare,Location="best");
stilizeazaLegenda(legendaEroare);
stilizeazaAxa(axaEroare);

fisierFiguraDistanta = fullfile(directorIesire,"figura_radar_distanta.png");
exportgraphics(figuraDistanta,fisierFiguraDistanta,Resolution=300);

fisierFiguraVitezaRadiala = fullfile(directorIesire, ...
    "figura_radar_eroare_viteza_radiala_vs_distanta.png");


%% Figura 2: eroarea vitezei radiale în funcție de distanță
figuraVitezaRadiala = figure(Name="Eroarea vitezei radiale Radar RSI",Color="w", ...
    Position=[120 90 1050 520],InvertHardcopy="off");
axaVitezaRadiala = axes(figuraVitezaRadiala);
mascaVitezaRadiala = masurariValide & isfinite(cadre.EroareViteza_mps) & ...
    isfinite(cadre.DistantaGT_m);
scatter(axaVitezaRadiala,cadre.DistantaGT_m(mascaVitezaRadiala), ...
    cadre.EroareViteza_mps(mascaVitezaRadiala),18,[0.10 0.45 0.85], ...
    "filled",MarkerFaceAlpha=0.55,DisplayName="Eroare viteză radială");
hold(axaVitezaRadiala,"on");
yline(axaVitezaRadiala,0,"--",Color=[0.25 0.25 0.25],LineWidth=1.3, ...
    DisplayName="Referință 0 m/s");
xlabel(axaVitezaRadiala,"Distanță ground truth (m)");
ylabel(axaVitezaRadiala,"Eroare viteză radială (m/s)");
title(axaVitezaRadiala,"Eroarea vitezei radiale în funcție de distanță");
legendaVitezaRadiala = legend(axaVitezaRadiala,Location="best");
stilizeazaLegenda(legendaVitezaRadiala);
stilizeazaAxa(axaVitezaRadiala);
exportgraphics(figuraVitezaRadiala,fisierFiguraVitezaRadiala,Resolution=300);

%% Figuri separate: calitatea detecției în funcție de distanță
fisierFiguraDisponibilitate = fullfile(directorIesire, ...
    "figura_radar_disponibilitate_vs_distanta.png");
fisierFiguraPrecizie = fullfile(directorIesire, ...
    "figura_radar_precizie_vs_distanta.png");
fisierFiguraRecall = fullfile(directorIesire, ...
    "figura_radar_recall_vs_distanta.png");
fisierFiguraF1 = fullfile(directorIesire, ...
    "figura_radar_f1_vs_distanta.png");

deseneazaIndicatorDistanta(centreDistanta_m,disponibilitateDistanta_pct, ...
    evaluabileDistanta,"Disponibilitatea detecției Radar RSI", ...
    "Disponibilitate (%)",[0.00 0.45 0.74],fisierFiguraDisponibilitate);
deseneazaIndicatorDistanta(centreDistanta_m,precizieDistanta_pct, ...
    evaluabileDistanta,"Precizia detecției Radar RSI", ...
    "Precizie (%)",[0.00 0.62 0.25],fisierFiguraPrecizie);
deseneazaIndicatorDistanta(centreDistanta_m,recallDistanta_pct, ...
    evaluabileDistanta,"Recall-ul detecției Radar RSI", ...
    "Recall (%)",[0.90 0.35 0.05],fisierFiguraRecall);
deseneazaIndicatorDistanta(centreDistanta_m,f1Distanta_pct, ...
    evaluabileDistanta,"Scorul F1 al detecției Radar RSI", ...
    "F1 (%)",[0.50 0.18 0.65],fisierFiguraF1);

fprintf("Cadre radar: %s\n",fisierCadre);
fprintf("Rezumat radar: %s\n",fisierRezumat);
fprintf("Calitate vs distanță: %s\n",fisierCalitateDistanta);
fprintf("Figura distanță: %s\n",fisierFiguraDistanta);
fprintf("Figura eroare viteză radială: %s\n",fisierFiguraVitezaRadiala);
fprintf("Figura disponibilitate: %s\n",fisierFiguraDisponibilitate);
fprintf("Figura precizie: %s\n",fisierFiguraPrecizie);
fprintf("Figura recall: %s\n",fisierFiguraRecall);
fprintf("Figura F1: %s\n",fisierFiguraF1);

function deseneazaIndicatorDistanta(centreDistanta_m,valori_pct,cadrePeInterval, ...
    titluFigura,etichetaY,culoare,fisierFigura)
figura = figure(Name=titluFigura,Color="w",Position=[120 90 1050 520], ...
    InvertHardcopy="off");
axa = axes(figura);
bar(axa,centreDistanta_m,valori_pct,0.82,FaceColor=culoare, ...
    EdgeColor=[0.08 0.08 0.08],LineWidth=0.9,DisplayName=etichetaY);
hold(axa,"on");
plot(axa,centreDistanta_m,valori_pct,"-o",LineWidth=2.0, ...
    MarkerSize=6,Color=[0.02 0.02 0.02],MarkerFaceColor=[1 1 1], ...
    DisplayName="Tendință pe distanță");
xlabel(axa,"Distanță ground truth (m)");
ylabel(axa,etichetaY);
title(axa,titluFigura);
ylim(axa,[0 105]);
valid = isfinite(valori_pct);
for i = find(valid(:)).'
    text(axa,centreDistanta_m(i),min(102,valori_pct(i)+3), ...
        sprintf("%.1f%%",valori_pct(i)),HorizontalAlignment="center", ...
        FontName="Times New Roman",FontSize=11,FontWeight="bold", ...
        Color=[0.05 0.05 0.05]);
    text(axa,centreDistanta_m(i),max(4,valori_pct(i)-8), ...
        sprintf("n=%d",cadrePeInterval(i)),HorizontalAlignment="center", ...
        FontName="Times New Roman",FontSize=10,Color=[0.05 0.05 0.05]);
end
legenda = legend(axa,Location="southoutside",Orientation="horizontal");
stilizeazaLegenda(legenda);
stilizeazaAxa(axa);
exportgraphics(figura,fisierFigura,Resolution=300);
end

function [timp_s,date] = extrageMatriceSemnal(semnal)
if isa(semnal,"timeseries")
    timp_s = double(semnal.Time(:));
    date = double(squeeze(semnal.Data));
elseif isstruct(semnal) && isfield(semnal,"time") && ...
        isfield(semnal,"signals")
    timp_s = double(semnal.time(:));
    date = double(squeeze(semnal.signals.values));
else
    error("Formatul variabilei radar_cadru_evaluare nu este recunoscut.");
end
if isvector(date)
    date = reshape(date,[],30);
end
if size(date,1)~=numel(timp_s) && size(date,2)==numel(timp_s)
    date = date.';
end
end

function valoare = procentSigur(numarator,numitor)
if numitor>0
    valoare = 100*double(numarator)/double(numitor);
else
    valoare = NaN;
end
end

function valoare = medieF1(precizie,recall)
if isfinite(precizie) && isfinite(recall) && precizie+recall>0
    valoare = 2*precizie*recall/(precizie+recall);
else
    valoare = NaN;
end
end

function valoare = medieSigura(date)
date = date(isfinite(date));
if isempty(date)
    valoare = NaN;
else
    valoare = mean(date);
end
end

function valoare = maeSigur(date)
date = date(isfinite(date));
if isempty(date)
    valoare = NaN;
else
    valoare = mean(abs(date));
end
end

function valoare = rmseSigur(date)
date = date(isfinite(date));
if isempty(date)
    valoare = NaN;
else
    valoare = sqrt(mean(date.^2));
end
end

function valoare = stdSigur(date)
date = date(isfinite(date));
if numel(date)<2
    valoare = NaN;
else
    valoare = std(date);
end
end

function stilizeazaAxa(axa)
grid(axa,"on");
box(axa,"on");
axa.Color = [1 1 1];
axa.FontName = "Times New Roman";
axa.FontSize = 13;
axa.FontWeight = "normal";
axa.LineWidth = 1.1;
axa.XColor = [0.12 0.12 0.12];
axa.GridColor = [0.72 0.72 0.72];
axa.GridAlpha = 0.45;
axa.MinorGridAlpha = 0.25;
axa.Layer = "top";
axa.Title.FontName = "Times New Roman";
axa.Title.FontSize = 15;
axa.Title.FontWeight = "bold";
axa.Title.Color = [0.08 0.08 0.08];
axa.XLabel.FontName = "Times New Roman";
axa.XLabel.FontSize = 14;
axa.XLabel.FontWeight = "bold";
axa.XLabel.Color = [0.08 0.08 0.08];
axa.YLabel.FontName = "Times New Roman";
axa.YLabel.FontSize = 14;
axa.YLabel.FontWeight = "bold";
if isscalar(axa.YAxis)
    axa.YColor = [0.12 0.12 0.12];
    axa.YLabel.Color = [0.08 0.08 0.08];
else
    axa.YAxis(1).Color = [0.38 0.12 0.52];
    axa.YAxis(2).Color = [0.22 0.45 0.10];
end
end

function stilizeazaLegenda(legenda)
legenda.FontName = "Times New Roman";
legenda.FontSize = 12;
legenda.TextColor = [0.08 0.08 0.08];
legenda.Color = [1 1 1];
legenda.EdgeColor = [0.35 0.35 0.35];
legenda.Box = "on";
end

function marcheazaNedetectii(axa,cadre)
masca = cadre.TintaPrezenta & ~cadre.AdevaratPozitiv;
if any(masca)
    scatter(axa,cadre.TimpRadar_s(masca),cadre.DistantaGT_m(masca), ...
        30,[0.64 0.08 0.18],"x",DisplayName="Țintă nedetectată");
end
end
