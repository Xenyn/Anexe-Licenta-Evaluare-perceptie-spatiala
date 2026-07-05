function [rezumat,cadre] = evalueaza_dataset_lidar(directorDataset,optiuni)
%EVALUEAZA_DATASET_LIDAR Evaluate a logged CarMaker LiDAR RSI experiment.
%   [REZUMAT,CADRE] = EVALUEAZA_DATASET_LIDAR(DIRECTOR) reads frame_####.mat
%   files produced by TestTCPMTL_logger_live_detection and evaluates target
%   availability, observed range, point density, angular resolution,
%   localization accuracy and temporal consistency.

arguments
    directorDataset (1,1) string {mustBeFolder}
    optiuni.GroundTruthDistance_m (1,1) double = NaN
    optiuni.GroundTruthSpeed_mps (1,1) double = NaN
    optiuni.GroundTruthOffset_m (1,1) double = NaN
    optiuni.GroundTruthPosition_m (1,3) double = [NaN NaN NaN]
    optiuni.GroundTruthDimensions_m (1,3) double = [NaN NaN NaN]
    optiuni.GroundTruthFile (1,1) string = ""
    optiuni.AssociationIoUThreshold (1,1) double = NaN
    optiuni.AssociationDistanceGate_m (1,1) double = NaN
    optiuni.GroundTruthReference (1,1) string ...
        {mustBeMember(optiuni.GroundTruthReference,["front_surface" "cluster_center"])} = "front_surface"
    optiuni.ExpectedVehicleCount (1,1) double {mustBeInteger,mustBeNonnegative} = 1
    optiuni.CreeazaGrafice (1,1) logical = true
end

fisiere = dir(fullfile(directorDataset,"frame_*.mat"));
nume = string({fisiere.name});
esteCadru = ~cellfun(@isempty,regexp(cellstr(nume),"^frame_\d+\.mat$","once"));
fisiere = fisiere(esteCadru);
[~,ordine] = sort(string({fisiere.name}));
fisiere = fisiere(ordine);

if isempty(fisiere)
    error("Nu exista fisiere frame_####.mat in %s.",directorDataset);
end

configExperiment = struct();
configLidar = struct();
tabelGroundTruth = table();
groundTruthIncarcat = false;
asocieri = table();
dateCadre = NaN(numel(fisiere),43);

for indiceCadru = 1:numel(fisiere)
    continut = load(fullfile(fisiere(indiceCadru).folder,fisiere(indiceCadru).name),"frameData");
    if ~isfield(continut,"frameData")
        continue;
    end
    cadru = continut.frameData;

    if isempty(fieldnames(configExperiment)) && isfield(cadru,"experimentConfig")
        configExperiment = cadru.experimentConfig;
    end
    if isempty(fieldnames(configLidar)) && isfield(cadru,"lidarConfig")
        configLidar = cadru.lidarConfig;
    end

    configurare = rezolvaConfigurare(configExperiment,optiuni);
    if ~groundTruthIncarcat
        tabelGroundTruth = incarcaGroundTruth(directorDataset,configurare,optiuni);
        groundTruthIncarcat = true;
    end
    vehicule = extrageVehicule(cadru);
    gtCadru = groundTruthPentruCadru(tabelGroundTruth,cadru,indiceCadru,configurare);
    [tinta,gtTinta,asocieriCadru,adevaratPozitive,falsPozitive,falsNegative] = ...
        asociazaDetectii(vehicule,gtCadru,configurare);
    if ~isempty(asocieriCadru)
        asocieriCadru.Cadru = repmat(valoareCamp(cadru,"frameID",indiceCadru), ...
            height(asocieriCadru),1);
        asocieriCadru.TimpCarMaker_s = repmat(valoareCamp(cadru,"carMakerTime_s",NaN), ...
            height(asocieriCadru),1);
        asocieri = [asocieri; asocieriCadru]; %#ok<AGROW>
    end

    numarPuncteBrute = size(cadru.xyz,1);
    distantePuncte = sqrt(sum(double(cadru.xyz).^2,2));
    distantaMaximaPuncte_m = max(distantePuncte,[],"omitmissing");
    numarVehicule = height(vehicule);
    groundTruthDisponibil = ~isempty(gtCadru);

    detectieValida = ~isempty(tinta);
    distantaEstimata_m = NaN;
    eroareSemnata_m = NaN;
    eroareSemnata_pct = NaN;
    numarPuncteTinta = NaN;
    densitatePuncte_m2 = NaN;
    numarPuncteCasetaGT = NaN;
    densitatePuncteGT_m2 = NaN;
    centruX_m = NaN;
    centruY_m = NaN;
    centruZ_m = NaN;
    intensitateMedie = NaN;
    eroareX_m = NaN;
    eroareY_m = NaN;
    eroareZ_m = NaN;
    eroareLocalizare3D_m = NaN;
    lungimeDetectata_m = NaN;
    latimeDetectata_m = NaN;
    inaltimeDetectata_m = NaN;
    iouBEV = NaN;
    iou3D = NaN;
    completitudineCluster_pct = NaN;
    [pozitieGT,dimensiuniGT,distantaGTCadru_m] = ...
        parametriGroundTruth(gtTinta,configurare);

    if all(isfinite(pozitieGT)) && all(isfinite(dimensiuniGT))
        numarPuncteCasetaGT = numaraPuncteInCaseta(double(cadru.xyz), ...
            pozitieGT,dimensiuniGT);
        arieVizibilaGT_m2 = max(dimensiuniGT(2)*dimensiuniGT(3),0.01);
        densitatePuncteGT_m2 = numarPuncteCasetaGT/arieVizibilaGT_m2;
    end

    if detectieValida
        if configurare.GroundTruthReference == "front_surface"
            distantaEstimata_m = tinta.MinX;
        else
            distantaEstimata_m = tinta.Range;
        end
        numarPuncteTinta = tinta.PointCount;
        suprafataFrontala_m2 = max(tinta.WidthY*tinta.HeightZ,0.01);
        densitatePuncte_m2 = numarPuncteTinta/suprafataFrontala_m2;
        centruX_m = tinta.CenterX;
        centruY_m = tinta.CenterY;
        centruZ_m = tinta.CenterZ;
        intensitateMedie = tinta.MeanIntensity;
        lungimeDetectata_m = tinta.LengthX;
        latimeDetectata_m = tinta.WidthY;
        inaltimeDetectata_m = tinta.HeightZ;

        if all(isfinite(pozitieGT))
            eroriPozitie = [centruX_m centruY_m centruZ_m] - ...
                pozitieGT;
            eroareX_m = eroriPozitie(1);
            eroareY_m = eroriPozitie(2);
            eroareZ_m = eroriPozitie(3);
            eroareLocalizare3D_m = norm(eroriPozitie);
        end

        if all(isfinite(pozitieGT)) && all(isfinite(dimensiuniGT))
            [iouBEV,iou3D,completitudineCluster_pct] = suprapunereCasete( ...
                [centruX_m centruY_m centruZ_m], ...
                [lungimeDetectata_m latimeDetectata_m inaltimeDetectata_m], ...
                pozitieGT,dimensiuniGT);
        end

        if isfinite(distantaGTCadru_m)
            eroareSemnata_m = distantaEstimata_m-distantaGTCadru_m;
            eroareSemnata_pct = 100*eroareSemnata_m/distantaGTCadru_m;
        end
    end

    dateCadre(indiceCadru,:) = [ ...
        valoareCamp(cadru,"frameID",indiceCadru) ...
        valoareCamp(cadru,"relayFrameNumber",NaN) ...
        valoareCamp(cadru,"carMakerTime_s",NaN) ...
        valoareCamp(cadru,"actualPointCount",numarPuncteBrute) ...
        valoareCamp(cadru,"sentPointCount",numarPuncteBrute) ...
        double(valoareCamp(cadru,"frameWasTruncated",false)) ...
        valoareCamp(cadru,"totalDroppedFrames",0) ...
        numarPuncteBrute distantaMaximaPuncte_m numarVehicule ...
        adevaratPozitive falsPozitive falsNegative ...
        double(groundTruthDisponibil) pozitieGT dimensiuniGT distantaGTCadru_m ...
        double(detectieValida) ...
        distantaEstimata_m eroareSemnata_m eroareSemnata_pct ...
        numarPuncteTinta densitatePuncte_m2 numarPuncteCasetaGT ...
        densitatePuncteGT_m2 centruX_m centruY_m centruZ_m ...
        intensitateMedie eroareX_m eroareY_m eroareZ_m eroareLocalizare3D_m ...
        lungimeDetectata_m latimeDetectata_m inaltimeDetectata_m ...
        iouBEV iou3D completitudineCluster_pct];

end

numeVariabile = ["Cadru" "CadruRelay" "TimpCarMaker_s" ...
    "NumarPuncteCarMaker" "NumarPuncteTransmise" "CadruTrunchiat" ...
    "CadreTCPPierduteCumulat" "NumarPuncteBrute" "DistantaMaximaPuncte_m" ...
    "NumarVehiculeCandidate" "AdevaratPozitive" "FalsePozitive" ...
    "FalseNegative" "GroundTruthDisponibil" ...
    "GT_X_m" "GT_Y_m" "GT_Z_m" "GT_Length_m" "GT_Width_m" ...
    "GT_Height_m" "DistantaGroundTruthCadru_m" "DetectieValida" ...
    "DistantaEstimata_m" "EroareSemnata_m" "EroareSemnata_pct" ...
    "NumarPuncteTinta" "DensitatePuncte_m2" ...
    "NumarPuncteCasetaGT" "DensitatePuncteGT_m2" ...
    "CentruX_m" "CentruY_m" "CentruZ_m" "IntensitateMedie" ...
    "EroareX_m" "EroareY_m" "EroareZ_m" "EroareLocalizare3D_m" ...
    "LungimeDetectata_m" "LatimeDetectata_m" "InaltimeDetectata_m" ...
    "IoU_BEV" "IoU_3D" "CompletitudineCluster_pct"];

cadre = array2table(dateCadre,VariableNames=numeVariabile);
cadre.DetectieValida = logical(cadre.DetectieValida);
cadre.GroundTruthDisponibil = logical(cadre.GroundTruthDisponibil);
cadre.CadruTrunchiat = logical(cadre.CadruTrunchiat);

configurare = rezolvaConfigurare(configExperiment,optiuni);
[rezolutieH_deg,rezolutieV_deg] = rezolutieAngulara(configLidar);
detectii = cadre.DetectieValida;
erori = cadre.EroareSemnata_m(detectii & isfinite(cadre.EroareSemnata_m));
distante = cadre.DistantaEstimata_m(detectii & isfinite(cadre.DistantaEstimata_m));
puncteTinta = cadre.NumarPuncteTinta(detectii & isfinite(cadre.NumarPuncteTinta));
densitati = cadre.DensitatePuncte_m2(detectii & isfinite(cadre.DensitatePuncte_m2));
puncteGT = cadre.NumarPuncteCasetaGT(isfinite(cadre.NumarPuncteCasetaGT));
densitatiGT = cadre.DensitatePuncteGT_m2(isfinite(cadre.DensitatePuncteGT_m2));
erori3D = cadre.EroareLocalizare3D_m(detectii & isfinite(cadre.EroareLocalizare3D_m));
totalTP = sum(cadre.AdevaratPozitive);
totalFP = sum(cadre.FalsePozitive);
totalFN = sum(cadre.FalseNegative);
precizieDetectie = raportSigur(totalTP,totalTP+totalFP);
recallDetectie = raportSigur(totalTP,totalTP+totalFN);
f1Detectie = raportSigur(2*precizieDetectie*recallDetectie, ...
    precizieDetectie+recallDetectie);
if precizieDetectie==0 && recallDetectie==0
    f1Detectie = 0;
end
distantaGTrezumat_m = configurare.GroundTruthDistance_m;
if (isfield(configurare,"GroundTruthSpeed_mps") && abs(configurare.GroundTruthSpeed_mps) > eps) || ~isfinite(distantaGTrezumat_m)
    distantaGTrezumat_m = medieSigura(cadre.DistantaGroundTruthCadru_m);
end

rezumat = table( ...
    string(configurare.ScenarioName),string(configurare.TrafficCondition), ...
    string(configurare.WeatherCondition),string(configurare.ObjectType), ...
    string(configurare.TargetMaterial),configurare.TargetReflectivity, ...
    string(configurare.TargetColor),configurare.TargetOrientation_deg, ...
    configurare.Visibility_m, ...
    configurare.RainIntensity,distantaGTrezumat_m, ...
    string(configurare.GroundTruthReference),string(configurare.GroundTruthFile), ...
    configurare.AssociationIoUThreshold,configurare.AssociationDistanceGate_m, ...
    height(cadre),100*recallDetectie,max(cadre.DistantaMaximaPuncte_m,[],"omitmissing"), ...
    medieSigura(distante),maxSigur(distante), ...
    medieSigura(puncteTinta),medieSigura(densitati), ...
    medieSigura(puncteGT),medieSigura(densitatiGT), ...
    rezolutieH_deg,rezolutieV_deg,medieSigura(erori), ...
    medieSigura(abs(erori)),sqrt(medieSigura(erori.^2)), ...
    medieSigura(erori3D),sqrt(medieSigura(erori3D.^2)), ...
    100*precizieDetectie,100*recallDetectie,100*f1Detectie, ...
    medieSigura(cadre.IoU_BEV),medieSigura(cadre.IoU_3D), ...
    medieSigura(cadre.CompletitudineCluster_pct), ...
    abatereSigura(distante),coeficientVariatie(puncteTinta), ...
    ceaMaiLungaPauza(~detectii),sum(cadre.FalsePozitive), ...
    100*mean(cadre.CadruTrunchiat),max(cadre.CadreTCPPierduteCumulat,[],"omitmissing"), ...
    VariableNames=["Scenariu" "ConditieTrafic" "ConditieMediu" "TipObiect" ...
    "MaterialTinta" "ReflectivitateTinta" "CuloareTinta" "OrientareTinta_deg" ...
    "Vizibilitate_m" "IntensitatePloaie" "DistantaGroundTruth_m" ...
    "ReferintaGroundTruth" "FisierGroundTruth" "PragAsociereIoU" ...
    "PoartaAsociereDistanta_m" "NumarCadre" "DisponibilitateDetectie_pct" ...
    "DistantaMaximaPuncteObservata_m" "DistantaMedieTinta_m" ...
    "DistantaMaximaTintaObservata_m" "NumarMediuPuncteTinta" ...
    "DensitateMediePuncte_m2" "NumarMediuPuncteCasetaGT" ...
    "DensitateMediePuncteGT_m2" "RezolutieOrizontala_deg" ...
    "RezolutieVerticala_deg" "Bias_m" "MAE_m" "RMSE_m" ...
    "MAELocalizare3D_m" "RMSELocalizare3D_m" ...
    "PrecizieDetectie_pct" "RecallDetectie_pct" "F1Detectie_pct" ...
    "IoUMediu_BEV" "IoUMediu_3D" "CompletitudineMedieCluster_pct" ...
    "JitterDistanta_m" "CVPuncteTinta" "PauzaMaximaDetectie_cadre" ...
    "TotalFalsePozitive" "CadreTrunchiate_pct" "CadreTCPPierdute"]);

writetable(cadre,fullfile(directorDataset,"cadre_evaluare_lidar.csv"));
writetable(rezumat,fullfile(directorDataset,"rezumat_evaluare_lidar.csv"));
if ~isempty(asocieri)
    writetable(asocieri,fullfile(directorDataset,"asocieri_evaluare_lidar.csv"));
end
save(fullfile(directorDataset,"evaluare_lidar.mat"),"cadre","rezumat", ...
    "asocieri","configurare");

if optiuni.CreeazaGrafice
    creeazaGrafice(cadre,rezumat,configurare,directorDataset);
end
end

function configurare = rezolvaConfigurare(configCadru,optiuni)
configurare = struct(ScenarioName="dataset_lidar",GroundTruthDistance_m=NaN, ...
    GroundTruthSpeed_mps=0, ...
    GroundTruthOffset_m=0, ...
    GroundTruthPosition_m=[NaN NaN NaN], ...
    GroundTruthDimensions_m=[NaN NaN NaN],GroundTruthFile="", ...
    ObjectType="nespecificat",TargetMaterial="nespecificat", ...
    TargetReflectivity=NaN,TargetColor="nespecificat",TargetOrientation_deg=NaN, ...
    AssociationIoUThreshold=0.30,AssociationDistanceGate_m=2.0, ...
    GroundTruthReference="front_surface",ExpectedVehicleCount=1, ...
    TrafficCondition="nespecificat",WeatherCondition="nespecificat", ...
    Visibility_m=NaN,RainIntensity=NaN);
nume = fieldnames(configCadru);
for k = 1:numel(nume)
    configurare.(nume{k}) = configCadru.(nume{k});
end
if isfinite(optiuni.GroundTruthDistance_m)
    configurare.GroundTruthDistance_m = optiuni.GroundTruthDistance_m;
end
if isfinite(optiuni.GroundTruthSpeed_mps)
    configurare.GroundTruthSpeed_mps = optiuni.GroundTruthSpeed_mps;
end
if isfinite(optiuni.GroundTruthOffset_m)
    configurare.GroundTruthOffset_m = optiuni.GroundTruthOffset_m;
end
if all(isfinite(optiuni.GroundTruthPosition_m))
    configurare.GroundTruthPosition_m = optiuni.GroundTruthPosition_m;
end
if all(isfinite(optiuni.GroundTruthDimensions_m))
    configurare.GroundTruthDimensions_m = optiuni.GroundTruthDimensions_m;
end
if strlength(optiuni.GroundTruthFile)>0
    configurare.GroundTruthFile = optiuni.GroundTruthFile;
end
if isfinite(optiuni.AssociationIoUThreshold)
    configurare.AssociationIoUThreshold = optiuni.AssociationIoUThreshold;
end
if isfinite(optiuni.AssociationDistanceGate_m)
    configurare.AssociationDistanceGate_m = optiuni.AssociationDistanceGate_m;
end
configurare.GroundTruthReference = optiuni.GroundTruthReference;
configurare.ExpectedVehicleCount = optiuni.ExpectedVehicleCount;
end

function vehicule = extrageVehicule(cadru)
vehicule = table();
if isfield(cadru,"vehicleTable") && istable(cadru.vehicleTable)
    vehicule = cadru.vehicleTable;
elseif isfield(cadru,"detectionTable") && istable(cadru.detectionTable) && ...
        ismember("VehicleCandidate",string(cadru.detectionTable.Properties.VariableNames))
    vehicule = cadru.detectionTable(cadru.detectionTable.VehicleCandidate==1,:);
end
end

function tabelGT = incarcaGroundTruth(directorDataset,configurare,optiuni)
tabelGT = table();
cale = optiuni.GroundTruthFile;
if strlength(cale)==0 && isfield(configurare,"GroundTruthFile")
    cale = string(configurare.GroundTruthFile);
end
if strlength(cale)==0
    caleImplicita = fullfile(directorDataset,"ground_truth_lidar.csv");
    if isfile(caleImplicita), cale=caleImplicita; end
elseif ~isfile(cale)
    cale = fullfile(directorDataset,cale);
end
if strlength(cale)>0 && isfile(cale)
    tabelGT = readtable(cale,TextType="string",VariableNamingRule="preserve");
    valideazaGroundTruth(tabelGT);
end
end

function valideazaGroundTruth(tabelGT)
nume = string(tabelGT.Properties.VariableNames);
areCheie = ismember("Cadru",nume) || ismember("TimpCarMaker_s",nume);
arePozitie = all(ismember(["GT_X_m" "GT_Y_m" "GT_Z_m"],nume));
areDistanta = ismember("GT_Distance_m",nume);
if ~areCheie || (~arePozitie && ~areDistanta)
    error(["Fisierul ground truth trebuie sa contina Cadru sau TimpCarMaker_s " ...
        "si pozitia GT_X_m/GT_Y_m/GT_Z_m sau GT_Distance_m."]);
end
end

function gtCadru = groundTruthPentruCadru(tabelGT,cadru,indiceCadru,configurare)
gtCadru = table();
if ~isempty(tabelGT)
    nume = string(tabelGT.Properties.VariableNames);
    if ismember("Cadru",nume)
        cadruCurent = valoareCamp(cadru,"frameID",indiceCadru);
        gtCadru = tabelGT(tabelGT.Cadru==cadruCurent,:);
    elseif ismember("TimpCarMaker_s",nume)
        timp = valoareCamp(cadru,"carMakerTime_s",NaN);
        if isfinite(timp)
            abatere = abs(tabelGT.TimpCarMaker_s-timp);
            abatereMinima = min(abatere,[],"omitmissing");
            gtCadru = tabelGT(abs(abatere-abatereMinima)<=1e-6,:);
        end
    elseif indiceCadru<=height(tabelGT)
        gtCadru = tabelGT(indiceCadru,:);
    end
end

distantaGTCadru = configurare.GroundTruthDistance_m;
if isfield(configurare,"GroundTruthSpeed_mps") && isfinite(configurare.GroundTruthSpeed_mps)
    timpCadru = valoareCamp(cadru,"carMakerTime_s",NaN);
    if isfinite(timpCadru) && isfinite(distantaGTCadru)
        distantaGTCadru = distantaGTCadru + configurare.GroundTruthSpeed_mps * timpCadru;
    end
end


if isfield(configurare,"GroundTruthOffset_m") && isfinite(configurare.GroundTruthOffset_m) && isfinite(distantaGTCadru)
    distantaGTCadru = distantaGTCadru + configurare.GroundTruthOffset_m;
end
if isempty(gtCadru) && (isfinite(distantaGTCadru) || ...
        all(isfinite(configurare.GroundTruthPosition_m)))
    gtCadru = table("dynamic",configurare.GroundTruthPosition_m(1), ...
        configurare.GroundTruthPosition_m(2),configurare.GroundTruthPosition_m(3), ...
        configurare.GroundTruthDimensions_m(1),configurare.GroundTruthDimensions_m(2), ...
        configurare.GroundTruthDimensions_m(3),distantaGTCadru, ...
        VariableNames=["ObjectID" "GT_X_m" "GT_Y_m" "GT_Z_m" ...
        "GT_Length_m" "GT_Width_m" "GT_Height_m" "GT_Distance_m"]);
end
end

function [tinta,gtTinta,asocieri,tp,fp,fn] = ...
        asociazaDetectii(vehicule,gtCadru,configurare)
tinta = table();
gtTinta = table();
asocieri = table();
if ~isempty(vehicule)
    vehicule = vehicule(vehicule.CenterX>0,:);
end

if isempty(gtCadru)
    tp = min(height(vehicule),configurare.ExpectedVehicleCount);
    fp = max(0,height(vehicule)-tp);
    fn = max(0,configurare.ExpectedVehicleCount-tp);
    if ~isempty(vehicule)
        [~,indice] = min(vehicule.Range);
        tinta = vehicule(indice,:);
    end
    return;
end

nGT = height(gtCadru);
nDetectii = height(vehicule);
detectieFolosita = false(nDetectii,1);
potrivita = false(nGT,1);
indiceDetectie = NaN(nGT,1);
distantaAsociere = NaN(nGT,1);
iouAsociere = NaN(nGT,1);
eroareDistanta = NaN(nGT,1);

for indiceGT = 1:nGT
    [pozitieGT,dimensiuniGT,distantaGT] = parametriGroundTruth( ...
        gtCadru(indiceGT,:),configurare);
    scor = Inf(nDetectii,1);
    for indiceDetectieCurenta = 1:nDetectii
        if detectieFolosita(indiceDetectieCurenta), continue; end
        detectie = vehicule(indiceDetectieCurenta,:);
        pozitieDetectata = [detectie.CenterX detectie.CenterY detectie.CenterZ];
        dimensiuniDetectate = [detectie.LengthX detectie.WidthY detectie.HeightZ];
        eroarePozitie = NaN;
        iou = NaN;
        if all(isfinite(pozitieGT))
            eroarePozitie = norm(pozitieDetectata-pozitieGT);
        end
        if all(isfinite(pozitieGT)) && all(isfinite(dimensiuniGT))
            [iou,~,~] = suprapunereCasete(pozitieDetectata, ...
                dimensiuniDetectate,pozitieGT,dimensiuniGT);
        end
        distantaDetectata = distantaDetectie(detectie,configurare.GroundTruthReference);
        eroareDist = abs(distantaDetectata-distantaGT);
        trecePoarta = (isfinite(iou) && iou>=configurare.AssociationIoUThreshold) || ...
            (isfinite(eroarePozitie) && eroarePozitie<=configurare.AssociationDistanceGate_m) || ...
            (~isfinite(eroarePozitie) && isfinite(eroareDist) && ...
            eroareDist<=configurare.AssociationDistanceGate_m);
        if trecePoarta
            criterii = [eroarePozitie/configurare.AssociationDistanceGate_m, ...
                eroareDist/configurare.AssociationDistanceGate_m,1-iou];
            scor(indiceDetectieCurenta) = min(criterii,[],"omitmissing");
        end
    end
    [scorMinim,indiceAles] = min(scor);
    if isfinite(scorMinim)
        potrivita(indiceGT) = true;
        indiceDetectie(indiceGT) = indiceAles;
        detectieFolosita(indiceAles) = true;
        detectie = vehicule(indiceAles,:);
        distantaAsociere(indiceGT) = norm( ...
            [detectie.CenterX detectie.CenterY detectie.CenterZ]-pozitieGT);
        if all(isfinite(pozitieGT)) && all(isfinite(dimensiuniGT))
            [iouAsociere(indiceGT),~,~] = suprapunereCasete( ...
                [detectie.CenterX detectie.CenterY detectie.CenterZ], ...
                [detectie.LengthX detectie.WidthY detectie.HeightZ], ...
                pozitieGT,dimensiuniGT);
        end
        eroareDistanta(indiceGT) = distantaDetectie(detectie, ...
            configurare.GroundTruthReference)-distantaGT;
    end
end

tp = nnz(potrivita);
fp = nDetectii-tp;
fn = nGT-tp;
objectID = string((1:nGT)');
if ismember("ObjectID",string(gtCadru.Properties.VariableNames))
    objectID = string(gtCadru.ObjectID);
end
asocieri = table(objectID,potrivita,indiceDetectie,distantaAsociere, ...
    iouAsociere,eroareDistanta,VariableNames=["ObjectID" "Potrivita" ...
    "IndiceDetectie" "EroarePozitie3D_m" "IoU_BEV" "EroareDistanta_m"]);

indicePrima = find(potrivita,1,"first");
if ~isempty(indicePrima)
    tinta = vehicule(indiceDetectie(indicePrima),:);
    gtTinta = gtCadru(indicePrima,:);
elseif ~isempty(gtCadru)
    gtTinta = gtCadru(1,:);
end
end

function [pozitie,dimensiuni,distanta] = parametriGroundTruth(gt,configurare)
pozitie = configurare.GroundTruthPosition_m;
dimensiuni = configurare.GroundTruthDimensions_m;
distanta = configurare.GroundTruthDistance_m;
if isempty(gt), return; end
nume = string(gt.Properties.VariableNames);
if all(ismember(["GT_X_m" "GT_Y_m" "GT_Z_m"],nume))
    pozitie = [gt.GT_X_m(1) gt.GT_Y_m(1) gt.GT_Z_m(1)];
end
if all(ismember(["GT_Length_m" "GT_Width_m" "GT_Height_m"],nume))
    dimensiuni = [gt.GT_Length_m(1) gt.GT_Width_m(1) gt.GT_Height_m(1)];
end
if ismember("GT_Distance_m",nume)
    distanta = gt.GT_Distance_m(1);
elseif all(isfinite(pozitie)) && all(isfinite(dimensiuni)) && ...
        configurare.GroundTruthReference=="front_surface"
    distanta = pozitie(1)-dimensiuni(1)/2;
elseif all(isfinite(pozitie))
    distanta = norm(pozitie);
end
end

function distanta = distantaDetectie(detectie,referinta)
if referinta=="front_surface"
    distanta = detectie.MinX;
else
    distanta = detectie.Range;
end
end

function numar = numaraPuncteInCaseta(xyz,centru,dimensiuni)
limitaMinima = centru-dimensiuni/2;
limitaMaxima = centru+dimensiuni/2;
inInterior = all(xyz>=limitaMinima & xyz<=limitaMaxima,2);
numar = nnz(inInterior);
end

function valoare = valoareCamp(structura,nume,implicit)
if isfield(structura,nume)
    valoare = double(structura.(nume));
else
    valoare = double(implicit);
end
end

function [orizontal,vertical] = rezolutieAngulara(config)
orizontal = NaN; vertical = NaN;
if isfield(config,"FoV_h") && isfield(config,"FoV_v") && isfield(config,"Beams_hv")
    divizor = config.Beams_hv;
    if isfield(config,"BeamSampling") && string(config.BeamSampling)=="endpoints"
        divizor = max(config.Beams_hv-1,1);
    end
    orizontal = diff(config.FoV_h)/divizor(1);
    vertical = diff(config.FoV_v)/divizor(2);
end
end

function valoare = medieSigura(x)
x = x(isfinite(x));
if isempty(x), valoare=NaN; else, valoare=mean(x); end
end

function valoare = maxSigur(x)
x = x(isfinite(x));
if isempty(x), valoare=NaN; else, valoare=max(x); end
end

function valoare = abatereSigura(x)
x = x(isfinite(x));
if numel(x)<2, valoare=NaN; else, valoare=std(x); end
end

function valoare = coeficientVariatie(x)
x = x(isfinite(x));
if numel(x)<2 || mean(x)==0, valoare=NaN; else, valoare=std(x)/mean(x); end
end

function valoare = raportSigur(numarator,numitor)
if ~isfinite(numitor) || numitor <= 0
    valoare = NaN;
else
    valoare = numarator/numitor;
end
end

function [iouBEV,iou3D,completitudine_pct] = suprapunereCasete( ...
        centruDetectat,dimensiuniDetectate,centruGT,dimensiuniGT)
minDetectat = centruDetectat-dimensiuniDetectate/2;
maxDetectat = centruDetectat+dimensiuniDetectate/2;
minGT = centruGT-dimensiuniGT/2;
maxGT = centruGT+dimensiuniGT/2;
intersectie = max(0,min(maxDetectat,maxGT)-max(minDetectat,minGT));

arieIntersectie = prod(intersectie(1:2));
arieDetectata = prod(dimensiuniDetectate(1:2));
arieGT = prod(dimensiuniGT(1:2));
iouBEV = raportSigur(arieIntersectie,arieDetectata+arieGT-arieIntersectie);

volumIntersectie = prod(intersectie);
volumDetectat = prod(dimensiuniDetectate);
volumGT = prod(dimensiuniGT);
iou3D = raportSigur(volumIntersectie,volumDetectat+volumGT-volumIntersectie);
completitudine_pct = 100*raportSigur(volumIntersectie,volumGT);
end

function lungime = ceaMaiLungaPauza(masca)
masca = logical(masca(:));
schimbari = diff([false;masca;false]);
inceput = find(schimbari==1);
sfarsit = find(schimbari==-1)-1;
if isempty(inceput), lungime=0; else, lungime=max(sfarsit-inceput+1); end
end

function creeazaGrafice(cadre,rezumat,configurare,directorDataset)
% Grafice finale pentru metodologia LiDAR: probabilitatea detecției,
% eroarea de distanță/MAE/RMSE și densitatea norului de puncte.

latimeBin_m = 10;
distantaGT = cadre.DistantaGroundTruthCadru_m;
detectie = logical(cadre.DetectieValida);

culoareDetectie = [0.10 0.50 0.90];
culoareEroare = [0.929 0.494 0.125];
culoareMAE = [0.00 0.60 0.25];
culoareRMSE = [0.49 0.18 0.56];
culoareDensitate = [0.494 0.184 0.556];
culoareMedie = [0.85 0.10 0.10];

[centreDistanta,probabilitateDetectie,eroareMedie,maeDistanta,rmseDistanta,densitateMedie,puncteMedii] = ...
    indicatoriMetodologiePeDistanta(cadre,latimeBin_m);

%% Figura 1: probabilitatea detecției în funcție de distanță
fig1 = figure(Name="LiDAR - probabilitatea detecției",Color="w",Position=[120 90 1200 560]);
ax1 = axes(fig1);
if ~isempty(centreDistanta)
    plot(ax1,centreDistanta,probabilitateDetectie,"-o",Color=culoareDetectie, ...
        LineWidth=2.2,MarkerSize=7,DisplayName="Probabilitatea detecției");
    lgd = legend(ax1,Location="southwest",FontSize=12);
    seteazaLegendaNeagra(lgd);
else
    text(ax1,0.5,0.5,"Nu există suficiente date pentru probabilitatea detecției", ...
        Units="normalized",HorizontalAlignment="center",FontSize=14,Color="k");
end
ylim(ax1,[0 105]); grid(ax1,"on"); box(ax1,"on");
xlabel(ax1,"Distanța ground truth (m)"); ylabel(ax1,"Probabilitatea detecției (%)");
title(ax1,"Probabilitatea detecției în funcție de distanță",FontSize=17,FontWeight="bold",Color="k");
stilizeazaAxa(ax1);
exportgraphics(fig1,fullfile(directorDataset,"lidar_01_probabilitate_detectie.png"),Resolution=300);

%% Figura 2: eroarea, MAE si RMSE în funcție de distanță
fig2 = figure(Name="LiDAR - eroare MAE RMSE",Color="w",Position=[140 100 1200 760]);
t2 = tiledlayout(fig2,2,1,TileSpacing="compact",Padding="compact");
title(t2,"Eroarea distanței LiDAR în funcție de distanță",FontSize=18,FontWeight="bold",Color="k");

ax2 = nexttile(t2);
mascaEroare = isfinite(distantaGT) & isfinite(cadre.EroareSemnata_m) & detectie;
if any(mascaEroare)
    scatter(ax2,distantaGT(mascaEroare),cadre.EroareSemnata_m(mascaEroare),38,culoareEroare, ...
        "filled",MarkerFaceAlpha=0.65,DisplayName="Eroare pe cadru");
    hold(ax2,"on");
end
if ~isempty(centreDistanta)
    plot(ax2,centreDistanta,eroareMedie,"-",Color=culoareMedie,LineWidth=2.2,DisplayName="Eroare medie pe interval");
end
yline(ax2,0,"--",Color=[0.15 0.15 0.15],LineWidth=1.2,DisplayName="Referință 0 m");
grid(ax2,"on"); box(ax2,"on");
xlabel(ax2,"Distanța ground truth (m)"); ylabel(ax2,"Eroare distanță (m)");
title(ax2,"Eroarea estimării distanței",Color="k");
lgd = legend(ax2,Location="best",FontSize=12); seteazaLegendaNeagra(lgd);
stilizeazaAxa(ax2);

ax3 = nexttile(t2);
if ~isempty(centreDistanta)
    plot(ax3,centreDistanta,maeDistanta,"-s",Color=culoareMAE,LineWidth=2.2,MarkerSize=7,DisplayName="MAE");
    hold(ax3,"on");
    plot(ax3,centreDistanta,rmseDistanta,"-d",Color=culoareRMSE,LineWidth=2.2,MarkerSize=7,DisplayName="RMSE");
    lgd = legend(ax3,Location="best",FontSize=12);
    seteazaLegendaNeagra(lgd);
else
    text(ax3,0.5,0.5,"Nu există suficiente date pentru MAE/RMSE pe distanță", ...
        Units="normalized",HorizontalAlignment="center",FontSize=14,Color="k");
end
grid(ax3,"on"); box(ax3,"on");
xlabel(ax3,"Distanța ground truth (m)"); ylabel(ax3,"Eroare absolută (m)");
title(ax3,sprintf("MAE/RMSE pe intervale de distanță | MAE global %.3f m | RMSE global %.3f m", ...
    rezumat.MAE_m,rezumat.RMSE_m),Color="k");
stilizeazaAxa(ax3);
exportgraphics(fig2,fullfile(directorDataset,"lidar_02_eroare_mae_rmse.png"),Resolution=300);

%% Figura 3: densitatea norului de puncte în funcție de distanță
fig3 = figure(Name="LiDAR - densitatea norului de puncte",Color="w",Position=[160 110 1200 560]);
ax4 = axes(fig3);
mascaDensitate = isfinite(distantaGT) & isfinite(cadre.DensitatePuncte_m2) & detectie;
if any(mascaDensitate)
    scatter(ax4,distantaGT(mascaDensitate),cadre.DensitatePuncte_m2(mascaDensitate),42,culoareDensitate, ...
        "filled",MarkerFaceAlpha=0.65,DisplayName="Densitate pe cadru");
    hold(ax4,"on");
end
if ~isempty(centreDistanta)
    plot(ax4,centreDistanta,densitateMedie,"-o",Color=culoareMedie,LineWidth=2.2,MarkerSize=7, ...
        DisplayName="Densitate medie pe interval");
end
grid(ax4,"on"); box(ax4,"on");
xlabel(ax4,"Distanța ground truth (m)"); ylabel(ax4,"Densitate (puncte/m²)");
title(ax4,"Densitatea norului de puncte în funcție de distanță",FontSize=17,FontWeight="bold",Color="k");
lgd = legend(ax4,Location="best",FontSize=12); seteazaLegendaNeagra(lgd);
stilizeazaAxa(ax4);
exportgraphics(fig3,fullfile(directorDataset,"lidar_03_densitate_nor_puncte.png"),Resolution=300);


%% Figura 4: puncte pe obiect și probabilitatea detecției
fig4 = figure(Name="LiDAR - puncte și probabilitatea detecției",Color="w",Position=[180 120 1200 560]);
ax5 = axes(fig4);
if ~isempty(centreDistanta)
    puncteNormalizate = 100 * puncteMedii ./ max(puncteMedii,[],"omitmissing");

    yyaxis(ax5,"left");
    hPuncte = plot(ax5,centreDistanta,puncteNormalizate,"-o",Color=[0.00 0.60 0.25], ...
        LineWidth=2.2,MarkerSize=7,DisplayName="Puncte pe obiect normalizate");
    ylabel(ax5,"Puncte pe obiect normalizate (%)",Color="k");
    ylim(ax5,[0 105]);
    ax5.YColor = "k";

    yyaxis(ax5,"right");
    hProb = plot(ax5,centreDistanta,probabilitateDetectie,"-s",Color=[0.10 0.50 0.90], ...
        LineWidth=2.2,MarkerSize=7,DisplayName="Probabilitatea detecției");
    ylabel(ax5,"Probabilitatea detecției (%)",Color="k");
    ylim(ax5,[0 105]);
    ax5.YColor = "k";
    lgd = legend(ax5,[hPuncte hProb],Location="best",FontSize=12);
    seteazaLegendaNeagra(lgd);
else
    text(ax5,0.5,0.5,"Nu există suficiente date pentru puncte/probabilitate", ...
        Units="normalized",HorizontalAlignment="center",FontSize=14,Color="k");
end
grid(ax5,"on"); box(ax5,"on");
xlabel(ax5,"Distanța ground truth (m)");
title(ax5,"Puncte pe obiect și probabilitatea detecției în funcție de distanță", ...
    FontSize=17,FontWeight="bold",Color="k");
yyaxis(ax5,"left"); ax5.YColor = "k";
yyaxis(ax5,"right"); ax5.YColor = "k";
stilizeazaAxa(ax5);
exportgraphics(fig4,fullfile(directorDataset,"lidar_04_puncte_probabilitate_detectie.png"),Resolution=300);

% Fișier generic pentru compatibilitate cu workflow-ul vechi.
exportgraphics(fig2,fullfile(directorDataset,"evaluare_lidar.png"),Resolution=300);
end
function stilizeazaAxa(ax)
ax.FontSize = 13;
ax.XColor = "k";
ax.YColor = "k";
ax.Title.Color = "k";
ax.XLabel.Color = "k";
ax.YLabel.Color = "k";
ax.Color = "w";
ax.LineWidth = 1.0;
ax.GridAlpha = 0.25;
ax.MinorGridAlpha = 0.12;
ax.XMinorGrid = "on";
ax.YMinorGrid = "on";
end
function seteazaLegendaNeagra(lgd)
if isempty(lgd) || ~isvalid(lgd), return; end
lgd.TextColor = "k";
lgd.Color = "w";
lgd.EdgeColor = [0.15 0.15 0.15];
end
function [centre,disponibilitate,precizie,recall,f1] = indicatoriDetectiePeDistanta(cadre,latimeBin_m)
distanta = cadre.DistantaGroundTruthCadru_m;
masca = isfinite(distanta) & cadre.GroundTruthDisponibil;
if ~any(masca)
    centre = []; disponibilitate = []; precizie = []; recall = []; f1 = [];
    return;
end

minD = floor(min(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
maxD = ceil(max(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
margini = minD:latimeBin_m:maxD;
if numel(margini) < 2
    margini = [minD minD+latimeBin_m];
end

nBin = numel(margini)-1;
centre = margini(1:end-1) + latimeBin_m/2;
disponibilitate = NaN(1,nBin);
precizie = NaN(1,nBin);
recall = NaN(1,nBin);
f1 = NaN(1,nBin);

for k = 1:nBin
    inBin = masca & distanta >= margini(k) & distanta < margini(k+1);
    if k == nBin
        inBin = masca & distanta >= margini(k) & distanta <= margini(k+1);
    end
    if ~any(inBin), continue; end

    tp = sum(cadre.AdevaratPozitive(inBin));
    fp = sum(cadre.FalsePozitive(inBin));
    fn = sum(cadre.FalseNegative(inBin));
    nEvaluabile = sum(inBin);

    disponibilitate(k) = 100 * raportSigur(sum(cadre.DetectieValida(inBin)),nEvaluabile);
    precizie(k) = 100 * raportSigur(tp,tp+fp);
    recall(k) = 100 * raportSigur(tp,tp+fn);
    f1(k) = 100 * raportSigur(2*(precizie(k)/100)*(recall(k)/100),(precizie(k)/100)+(recall(k)/100));
end

valid = isfinite(disponibilitate) | isfinite(precizie) | isfinite(recall) | isfinite(f1);
centre = centre(valid);
disponibilitate = disponibilitate(valid);
precizie = precizie(valid);
recall = recall(valid);
f1 = f1(valid);
end
function [centre,falsePozitive] = falsePozitivePeDistanta(cadre,latimeBin_m)
distanta = cadre.DistantaGroundTruthCadru_m;
masca = isfinite(distanta) & cadre.GroundTruthDisponibil;
if ~any(masca)
    centre = []; falsePozitive = [];
    return;
end

minD = floor(min(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
maxD = ceil(max(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
margini = minD:latimeBin_m:maxD;
if numel(margini) < 2
    margini = [minD minD+latimeBin_m];
end

nBin = numel(margini)-1;
centre = margini(1:end-1) + latimeBin_m/2;
falsePozitive = NaN(1,nBin);

for k = 1:nBin
    inBin = masca & distanta >= margini(k) & distanta < margini(k+1);
    if k == nBin
        inBin = masca & distanta >= margini(k) & distanta <= margini(k+1);
    end
    if ~any(inBin), continue; end
    falsePozitive(k) = sum(cadre.FalsePozitive(inBin));
end

valid = isfinite(falsePozitive);
centre = centre(valid);
falsePozitive = falsePozitive(valid);
end
function [centre,probabilitateDetectie,eroareMedie,maeDistanta,rmseDistanta,densitateMedie,puncteMedii] = indicatoriMetodologiePeDistanta(cadre,latimeBin_m)
distanta = cadre.DistantaGroundTruthCadru_m;
masca = isfinite(distanta) & cadre.GroundTruthDisponibil;
if ~any(masca)
    centre = []; probabilitateDetectie = []; eroareMedie = []; maeDistanta = []; rmseDistanta = []; densitateMedie = []; puncteMedii = [];
    return;
end

minD = floor(min(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
maxD = ceil(max(distanta(masca),[],'omitmissing')/latimeBin_m)*latimeBin_m;
margini = minD:latimeBin_m:maxD;
if numel(margini) < 2
    margini = [minD minD+latimeBin_m];
end

nBin = numel(margini)-1;
centre = margini(1:end-1) + latimeBin_m/2;
probabilitateDetectie = NaN(1,nBin);
eroareMedie = NaN(1,nBin);
maeDistanta = NaN(1,nBin);
rmseDistanta = NaN(1,nBin);
densitateMedie = NaN(1,nBin);
puncteMedii = NaN(1,nBin);

for k = 1:nBin
    inBin = masca & distanta >= margini(k) & distanta < margini(k+1);
    if k == nBin
        inBin = masca & distanta >= margini(k) & distanta <= margini(k+1);
    end
    if ~any(inBin), continue; end

    probabilitateDetectie(k) = 100 * raportSigur(sum(cadre.DetectieValida(inBin)),sum(inBin));

    eroriBin = cadre.EroareSemnata_m(inBin & isfinite(cadre.EroareSemnata_m) & cadre.DetectieValida);
    if ~isempty(eroriBin)
        eroareMedie(k) = mean(eroriBin,'omitmissing');
        maeDistanta(k) = mean(abs(eroriBin),'omitmissing');
        rmseDistanta(k) = sqrt(mean(eroriBin.^2,'omitmissing'));
    end

    densitatiBin = cadre.DensitatePuncte_m2(inBin & isfinite(cadre.DensitatePuncte_m2) & cadre.DetectieValida);
    if ~isempty(densitatiBin)
        densitateMedie(k) = mean(densitatiBin,'omitmissing');
    end
end

valid = isfinite(probabilitateDetectie) | isfinite(eroareMedie) | isfinite(maeDistanta) | ...
    isfinite(rmseDistanta) | isfinite(densitateMedie) | isfinite(puncteMedii);
centre = centre(valid);
probabilitateDetectie = probabilitateDetectie(valid);
eroareMedie = eroareMedie(valid);
maeDistanta = maeDistanta(valid);
rmseDistanta = rmseDistanta(valid);
densitateMedie = densitateMedie(valid);
puncteMedii = puncteMedii(valid);
end





