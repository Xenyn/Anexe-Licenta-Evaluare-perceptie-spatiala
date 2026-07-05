clear; clc;

scriptDir = fileparts(mfilename("fullpath"));

%% ========================================================================
% TCP SETTINGS
% ========================================================================
host = "localhost";
port = 55000;

% Must match C++ MAX_POINTS.
MAX_POINTS = 300300;
VALUES_PER_POINT = 4;
HEADER_FLOATS = 9;
PACKET_MAGIC = single(1279345);
PACKET_VERSION = single(1);
N_FLOATS = HEADER_FLOATS + MAX_POINTS * VALUES_PER_POINT;
N_BYTES = N_FLOATS * 4;   % single = 4 bytes

t = tcpclient(host, port, "Timeout", 60);

%% ========================================================================
% VIEWER
% ========================================================================
ENABLE_VIEWER = usejava('desktop');

if ENABLE_VIEWER
    player = pcplayer([-5 250], [-50 50], [-50 50]);
    xlabel(player.Axes, "X [m]");
    ylabel(player.Axes, "Y [m]");
    zlabel(player.Axes, "Z [m]");
else
    player = [];
end

disp("Connected. Waiting for LiDAR frames...");

%% ========================================================================
% LOGGING SETTINGS
% ========================================================================
ENABLE_LOGGING = true;

testRunFile = "E:\License\Data\TestRun\TestS";
groundTruthDistance_m = readPrimaryTrafficStartDistance(testRunFile, 15);
groundTruthSpeed_mps = readPrimaryTrafficLongitudinalSpeed(testRunFile, 0);
if abs(groundTruthSpeed_mps) > eps
    speedTag = replace(string(sprintf("%.1f",groundTruthSpeed_mps)),".","p");
    scenarioNameAuto = "onevehicle" + string(round(groundTruthDistance_m)) + "m_dyn" + speedTag + "mps";
else
    scenarioNameAuto = "onevehicle" + string(round(groundTruthDistance_m)) + "m";
end

experimentConfig = struct( ...
    ScenarioName=scenarioNameAuto, ...
    GroundTruthDistance_m=groundTruthDistance_m, ...
    GroundTruthSpeed_mps=groundTruthSpeed_mps, ...
    GroundTruthReference="front_surface", ...
    GroundTruthPosition_m=[NaN NaN NaN], ... 
    GroundTruthDimensions_m=[NaN NaN NaN], ... 
    GroundTruthFile="", ... 
    ObjectType="vehicle", ...
    TargetMaterial="painted_metal", ...
    TargetReflectivity=NaN, ...
    TargetColor="unspecified", ...
    TargetOrientation_deg=0, ...
    AssociationIoUThreshold=0.30, ...
    AssociationDistanceGate_m=2.0, ...
    ExpectedVehicleCount=1, ...
    TrafficCondition="single_vehicle", ...
    WeatherCondition="clear", ...
    Visibility_m=Inf, ...
    RainIntensity=0);

scenarioName = experimentConfig.ScenarioName;
datasetRoot = fullfile(scriptDir, "lidar_dataset");
outDir = fullfile(datasetRoot, scenarioName);

if ENABLE_LOGGING && ~exist(outDir, "dir")
    mkdir(outDir);
end

if ENABLE_LOGGING
    stergeFisiereVechiDataset(outDir);
end

fprintf("LiDAR dataset output folder: %s\n", outDir);

SAVE_EVERY_N_FRAMES = 20;
MAX_SAVED_FRAMES = 100;

frameID = 0;
savedID = 0;
lastRelayFrameNumber = NaN;
totalDroppedFrames = 0;

lidarConfig = struct();
lidarConfig.FoV_h = [-60 60];
lidarConfig.FoV_v = [-12.5 12.5];
lidarConfig.Beams_hv = [1001 300];
lidarConfig.BeamSampling = "cell_centered";
lidarConfig.Rays_hv = [1 1];
lidarConfig.SensorPosition = [2.7 0 1.6];
lidarConfig.MAX_POINTS = MAX_POINTS;
lidarConfig.VALUES_PER_POINT = VALUES_PER_POINT;

%% ========================================================================
% LIVE DETECTION SETTINGS
% ========================================================================
ENABLE_LIVE_DETECTION = ENABLE_VIEWER;

% Live detection is expensive.
DETECT_EVERY_N_FRAMES = 20;

% If true, every saved frame also gets a detection table saved.
RUN_DETECTION_ON_SAVED_FRAMES = true;

detParams = getDetectionParams();

latestDetectionTable = table();
latestVehicleTable = table();
latestSignTable = table();

%% ========================================================================
% MAIN LOOP
% ========================================================================
MAX_HEADLESS_RUNTIME_s = 240;
loggerStartTime = tic;

while ((ENABLE_VIEWER && isOpen(player)) || ...
        (~ENABLE_VIEWER && savedID < MAX_SAVED_FRAMES)) && ...
        (ENABLE_VIEWER || toc(loggerStartTime) < MAX_HEADLESS_RUNTIME_s)
    if t.NumBytesAvailable < N_BYTES
        pause(0.01);
        continue;
    end

    rawBytes = read(t, N_BYTES, "uint8");
    raw = typecast(uint8(rawBytes), "single");

    if numel(raw) ~= N_FLOATS || raw(1) ~= PACKET_MAGIC || ...
            raw(2) ~= PACKET_VERSION || round(raw(9)) ~= VALUES_PER_POINT
        warning("Invalid LiDAR TCP packet header. Frame discarded.");
        continue;
    end

    carMakerTime_s = double(raw(3));
    relayFrameNumber = double(raw(4));
    sensorID = double(raw(5));
    actualPointCount = double(raw(6));
    sentPointCount = min(MAX_POINTS,max(0,round(double(raw(7)))));
    frameWasTruncated = logical(raw(8));

    if isfinite(lastRelayFrameNumber) && relayFrameNumber > lastRelayFrameNumber + 1
        droppedNow = relayFrameNumber-lastRelayFrameNumber-1;
        totalDroppedFrames = totalDroppedFrames+droppedNow;
        warning("Detected %d dropped LiDAR TCP frame(s).",droppedNow);
    end
    lastRelayFrameNumber = relayFrameNumber;

    if frameWasTruncated
        warning("LiDAR frame %d truncated: received %d of %d scan points.", ...
            relayFrameNumber,sentPointCount,actualPointCount);
    end

    payload = raw(HEADER_FLOATS+1:end);
    frame = reshape(payload,VALUES_PER_POINT,[]).';
    frame = frame(1:sentPointCount,:);

    xyz = frame(:,1:3);
    intensity = frame(:,4);

    valid = all(isfinite(xyz),2) & intensity > 0;

    xyz = xyz(valid,:);
    intensity = intensity(valid);

    if isempty(xyz)
        continue;
    end

    %% LOGGING / DETECTION COPY
    % Keep real LiDAR points before ego vehicle points are appended.
    xyzLog = xyz;
    intensityLog = intensity;
    frameID = frameID + 1;

    %% INTENSITY COLORING - same logic as working viewer
    i = double(intensity);
    i(~isfinite(i)) = 0;

    lo = prctile(i, 2);
    hi = prctile(i, 98);

    iNorm = (i - lo) ./ max(hi - lo, eps);
    iNorm = min(max(iNorm, 0), 1);

    cmap = turbo(256);
    idx = max(1, min(256, round(iNorm * 255) + 1));
    rgb = cmap(idx, :);

    %% EGO VEHICLE OUTLINE - same logic as working viewer
    xFront =  3.2;
    xRear  = -1.3;
    yLeft  =  0.9;
    yRight = -0.9;
    zLow   =  0.0;
    zHigh  =  1.5;

    corners = [
        xFront yLeft  zLow
        xFront yRight zLow
        xRear  yRight zLow
        xRear  yLeft  zLow
        xFront yLeft  zHigh
        xFront yRight zHigh
        xRear  yRight zHigh
        xRear  yLeft  zHigh
    ];

    edges = [
        1 2; 2 3; 3 4; 4 1
        5 6; 6 7; 7 8; 8 5
        1 5; 2 6; 3 7; 4 8
    ];

    egoPts = [];

    for e = 1:size(edges,1)
        a = corners(edges(e,1),:);
        b = corners(edges(e,2),:);

        for k = 0:20
            tLine = k/20;
            egoPts(end+1,:) = a + tLine*(b-a); %#ok<AGROW>
        end
    end

    sensorPt = [2.7 0 1.6];

    xyz = [xyz; egoPts; sensorPt];

    egoRgb = repmat([1 0 0], size(egoPts,1), 1);
    sensorRgb = [1 1 1];

    rgb = [rgb; egoRgb; sensorRgb];

    %% LIVE POINT CLOUD VIEW
    if ENABLE_VIEWER
        view(player, single(xyz), rgb);
    end

    %% LIVE DETECTION / SCANNING
    shouldRunLiveDetection = ENABLE_LIVE_DETECTION && ...
                             mod(frameID, DETECT_EVERY_N_FRAMES) == 0;

    if shouldRunLiveDetection
        [latestDetectionTable, latestVehicleTable, latestSignTable, latestPlotData] = ...
            detectObjectsFromLidarFrame(xyzLog, intensityLog, detParams);

        updateLiveDetectionFigure(latestPlotData, latestDetectionTable, frameID, detParams);
    end

    %% LOGGING
    if ENABLE_LOGGING && mod(frameID, SAVE_EVERY_N_FRAMES) == 0
        savedID = savedID + 1;

        frameData = struct();
        frameData.xyz = single(xyzLog);
        frameData.intensity = single(intensityLog);
        frameData.range = single(sqrt(sum(double(xyzLog).^2, 2)));
        frameData.frameID = frameID;
        frameData.relayFrameNumber = relayFrameNumber;
        frameData.carMakerTime_s = carMakerTime_s;
        frameData.sensorID = sensorID;
        frameData.actualPointCount = actualPointCount;
        frameData.sentPointCount = sentPointCount;
        frameData.frameWasTruncated = frameWasTruncated;
        frameData.totalDroppedFrames = totalDroppedFrames;
        frameData.savedID = savedID;
        frameData.scenario = scenarioName;
        frameData.experimentConfig = experimentConfig;
        frameData.lidarConfig = lidarConfig;

        % Run detection on saved frame if it was not already run this frame.
        detectionTable = latestDetectionTable;
        vehicleTable = latestVehicleTable;
        signTable = latestSignTable;

        if RUN_DETECTION_ON_SAVED_FRAMES && ~shouldRunLiveDetection
            [detectionTable, vehicleTable, signTable, latestPlotData] = ...
                detectObjectsFromLidarFrame(xyzLog, intensityLog, detParams);

            latestDetectionTable = detectionTable;
            latestVehicleTable = vehicleTable;
            latestSignTable = signTable;

            if ENABLE_LIVE_DETECTION
                updateLiveDetectionFigure(latestPlotData, detectionTable, frameID, detParams);
            end
        end

        frameData.detectionTable = detectionTable;
        frameData.vehicleTable = vehicleTable;
        frameData.signTable = signTable;
        frameData.detParams = detParams;

        fileName = fullfile(outDir, sprintf("frame_%04d.mat", savedID));
        save(fileName, "frameData");

        % Separate CSV detection report per saved frame.
        if ~isempty(detectionTable)
            csvName = fullfile(outDir, sprintf("frame_%04d_detections.csv", savedID));
            writetable(detectionTable, csvName);
        end

        fprintf("Saved %s | points = %d | vehicles = %d | signs = %d\n", ...
            fileName, size(xyzLog,1), height(vehicleTable), height(signTable));

        if savedID >= MAX_SAVED_FRAMES
            disp("Finished logging. Viewer will close.");
            break;
        end
    end
end

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function params = getDetectionParams()
    % ROI crop
    params.XLim = [-5 220];
    params.YLim = [-40 40];
    params.ZLim = [-3 10];

    % Ground removal
    params.GroundMaxDistance = 0.20;
    params.GroundNormal = [0 0 1];
    params.GroundMaxAngularDistance = 10;
    params.FallbackGroundZ = 0.25;

    % Clustering
    params.ClusterDistance = 1.8;
    params.MinClusterPoints = 20;

    % Live processing decimation
    params.MaxPointsForDetection = 120000;

    % Vehicle candidates
    params.VehicleXLim = [0 190];
    params.VehicleAbsYMax = 20;
    params.VehicleLongDimMin = 1.6;
    params.VehicleLongDimMax = 35.0;
    params.VehicleShortDimMax = 9.0;
    params.VehicleHeightMin = 0.8;
    params.VehicleHeightMax = 7.0;
    params.VehicleFaceWidthMin = 1.4;
    params.VehicleFaceHeightMin = 1.2;

    params.VehicleMinPointsNear = 120;
    params.VehicleMinPointsMid  = 60;
    params.VehicleMinPointsFar  = 25;

    % Sign candidates
    params.SignAbsYMax = 28;
    params.SignMaxLongDim = 3.0;
    params.SignMaxShortDim = 1.8;
    params.SignMaxFootprintArea = 4.0;
    params.SignMinHeight = 0.6;
    params.SignMaxHeight = 5.5;
    params.SignMinPointsNear = 8;
    params.SignMinPointsFar = 3;

    % Low-point support
    params.LowPointZMax = 1.2;
    params.VehicleMinLowPointRatio = 0.12;
    params.SignMaxLowPointRatio = 0.08;
    params.SignMinBaseHeight = 0.45;

    % Detection plot
    params.ViewXLim = [-5 250];
    params.ViewYLim = [-60 60];
    params.ViewZLim = [-5 25];
end

function [detectionTable, vehicleTable, signTable, plotData] = detectObjectsFromLidarFrame(xyz, intensity, params)

    detectionTable = table();
    vehicleTable = table();
    signTable = table();

    plotData = struct();
    plotData.vehCenters = [];
    plotData.signCenters = [];
    plotData.otherCenters = [];
    plotData.boxes = {};
    plotData.boxColors = {};

    valid = all(isfinite(xyz), 2) & isfinite(intensity) & intensity > 0 & intensity < 9000;

    xyz = double(xyz(valid, :));
    intensity = double(intensity(valid));

    if isempty(xyz)
        return;
    end

    roiMask = xyz(:,1) >= params.XLim(1) & xyz(:,1) <= params.XLim(2) & ...
              xyz(:,2) >= params.YLim(1) & xyz(:,2) <= params.YLim(2) & ...
              xyz(:,3) >= params.ZLim(1) & xyz(:,3) <= params.ZLim(2);

    xyzROI = xyz(roiMask, :);
    intensityROI = intensity(roiMask);

    if isempty(xyzROI)
        return;
    end

    % Decimate only the detection branch for live performance.
    if size(xyzROI, 1) > params.MaxPointsForDetection
        numarPuncte = size(xyzROI,1);
        idx = floor((0:params.MaxPointsForDetection-1)' .* ...
            numarPuncte ./ params.MaxPointsForDetection)+1;
        xyzROI = xyzROI(idx, :);
        intensityROI = intensityROI(idx);
    end

    ptROI = pointCloud(single(xyzROI), "Intensity", single(intensityROI));

    try
        [~, groundIdx] = pcfitplane( ...
            ptROI, ...
            params.GroundMaxDistance, ...
            params.GroundNormal, ...
            params.GroundMaxAngularDistance);

        nonGroundMask = true(ptROI.Count, 1);
        nonGroundMask(groundIdx) = false;

        xyzNG = xyzROI(nonGroundMask, :);
        intensityNG = intensityROI(nonGroundMask);

    catch
        nonGroundMask = xyzROI(:,3) > params.FallbackGroundZ;
        xyzNG = xyzROI(nonGroundMask, :);
        intensityNG = intensityROI(nonGroundMask);
    end

    if isempty(xyzNG)
        return;
    end

    ptNG = pointCloud(single(xyzNG), "Intensity", single(intensityNG));

    try
        [labels, numClusters] = pcsegdist(ptNG, params.ClusterDistance);
    catch
        return;
    end

    detections = [];

    for c = 1:numClusters
        clusterMask = labels == c;

        pts = xyzNG(clusterMask, :);
        inten = intensityNG(clusterMask);

        nPts = size(pts, 1);

        if nPts < params.MinClusterPoints
            continue;
        end

        minPt = min(pts, [], 1);
        maxPt = max(pts, [], 1);
        dims = maxPt - minPt;

        center = mean(pts, 1);
        range = norm(center);

        lengthX = dims(1);
        widthY  = dims(2);
        heightZ = dims(3);

        horizontalLong = max(lengthX, widthY);
        horizontalShort = min(lengthX, widthY);
        footprintArea = max(lengthX, 0.01) * max(widthY, 0.01);

        meanIntensity = mean(inten);
        medianIntensity = median(inten);
        maxIntensity = max(inten);

        minZ = minPt(3);
        maxZ = maxPt(3);
        lowPointRatio = sum(pts(:,3) <= params.LowPointZMax) / nPts;

        if range < 40
            minRequiredVehiclePoints = params.VehicleMinPointsNear;
        elseif range < 100
            minRequiredVehiclePoints = params.VehicleMinPointsMid;
        else
            minRequiredVehiclePoints = params.VehicleMinPointsFar;
        end

        if range < 80
            minRequiredSignPoints = params.SignMinPointsNear;
        else
            minRequiredSignPoints = params.SignMinPointsFar;
        end

        inVehicleRoadCorridor = center(1) >= params.VehicleXLim(1) && ...
                                center(1) <= params.VehicleXLim(2) && ...
                                abs(center(2)) <= params.VehicleAbsYMax;

        inSignCorridor = center(1) >= params.VehicleXLim(1) && ...
                         center(1) <= params.VehicleXLim(2) && ...
                         abs(center(2)) <= params.SignAbsYMax;

        smallFootprint = footprintArea <= params.SignMaxFootprintArea;
        narrowObject = horizontalShort <= params.SignMaxShortDim && ...
                       horizontalLong <= params.SignMaxLongDim;

        signHeightOK = heightZ >= params.SignMinHeight && ...
                       heightZ <= params.SignMaxHeight;

        elevatedLikeSign = minZ >= params.SignMinBaseHeight || ...
                           lowPointRatio <= params.SignMaxLowPointRatio;

        isTrafficSignCandidate = inSignCorridor && ...
                                 (smallFootprint || narrowObject) && ...
                                 signHeightOK && ...
                                 elevatedLikeSign && ...
                                 nPts >= minRequiredSignPoints;

        vehicleFootprint = horizontalLong >= params.VehicleLongDimMin && ...
                           horizontalLong <= params.VehicleLongDimMax && ...
                           horizontalShort <= params.VehicleShortDimMax && ...
                           heightZ >= params.VehicleHeightMin && ...
                           heightZ <= params.VehicleHeightMax;

        vehicleFace = widthY >= params.VehicleFaceWidthMin && ...
                      heightZ >= params.VehicleFaceHeightMin && ...
                      heightZ <= params.VehicleHeightMax && ...
                      horizontalLong <= params.VehicleLongDimMax;

        hasVehicleLowSupport = lowPointRatio >= params.VehicleMinLowPointRatio || ...
                               minZ <= params.LowPointZMax;

        isVehicleCandidate = inVehicleRoadCorridor && ...
                             (vehicleFootprint || vehicleFace) && ...
                             nPts >= minRequiredVehiclePoints && ...
                             hasVehicleLowSupport && ...
                             ~isTrafficSignCandidate;

        classCode = 0;

        if isVehicleCandidate
            classCode = 1;
            plotData.vehCenters = [plotData.vehCenters; center]; 
            plotData.boxes{end+1} = [minPt maxPt]; 
            plotData.boxColors{end+1} = "r"; 
        elseif isTrafficSignCandidate
            classCode = 2;
            plotData.signCenters = [plotData.signCenters; center]; 
            plotData.boxes{end+1} = [minPt maxPt]; 
            plotData.boxColors{end+1} = "m"; 
        else
            plotData.otherCenters = [plotData.otherCenters; center]; 
        end

        detections = [detections; ...
            c, nPts, ...
            center(1), center(2), center(3), ...
            range, ...
            lengthX, widthY, heightZ, ...
            horizontalLong, horizontalShort, footprintArea, ...
            meanIntensity, medianIntensity, maxIntensity, ...
            lowPointRatio, minZ, maxZ, ...
            minRequiredVehiclePoints, minRequiredSignPoints, ...
            double(isVehicleCandidate), double(isTrafficSignCandidate), classCode, ...
            minPt(1), minPt(2), minPt(3), ...
            maxPt(1), maxPt(2), maxPt(3)]; %#ok<AGROW>
    end

    if isempty(detections)
        return;
    end

    varNames = { ...
        'ClusterID', ...
        'PointCount', ...
        'CenterX', 'CenterY', 'CenterZ', ...
        'Range', ...
        'LengthX', 'WidthY', 'HeightZ', ...
        'HorizontalLong', 'HorizontalShort', 'FootprintArea', ...
        'MeanIntensity', 'MedianIntensity', 'MaxIntensity', ...
        'LowPointRatio', 'MinZCluster', 'MaxZCluster', ...
        'MinRequiredVehiclePoints', 'MinRequiredSignPoints', ...
        'VehicleCandidate', 'TrafficSignCandidate', 'ClassCode', ...
        'MinX', 'MinY', 'MinZ', ...
        'MaxX', 'MaxY', 'MaxZ'};

    detectionTable = array2table(detections, 'VariableNames', varNames);
    detectionTable = sortrows(detectionTable, 'Range');

    vehicleTable = detectionTable(detectionTable.VehicleCandidate == 1, :);
    signTable = detectionTable(detectionTable.TrafficSignCandidate == 1, :);
end

function updateLiveDetectionFigure(plotData, detectionTable, frameID, params)
    persistent fig ax hVeh hSign hOther hBoxes hTitle

    if isempty(fig) || ~isvalid(fig)
        fig = figure("Name", "Live LiDAR Object Scan", "NumberTitle", "off");
        ax = axes(fig);

        hold(ax, "on");
        grid(ax, "on");
        axis(ax, "equal");

        xlabel(ax, "X [m]");
        ylabel(ax, "Y [m]");
        zlabel(ax, "Z [m]");

        xlim(ax, params.ViewXLim);
        ylim(ax, params.ViewYLim);
        zlim(ax, params.ViewZLim);
        view(ax, 0, 90);

        hVeh = scatter3(ax, NaN, NaN, NaN, 100, "r", "filled");
        hSign = scatter3(ax, NaN, NaN, NaN, 80, "m", "filled");
        hOther = scatter3(ax, NaN, NaN, NaN, 35, "y", "filled");

        [egoPts, sensorPt] = makeDetectionEgoVehicle();
        scatter3(ax, egoPts(:,1), egoPts(:,2), egoPts(:,3), 12, "r", "filled");
        scatter3(ax, sensorPt(1), sensorPt(2), sensorPt(3), 80, "w", "filled");

        legend(ax, [hVeh hSign hOther], ...
            ["Vehicle candidates", "Traffic sign candidates", "Other clusters"], ...
            "Location", "northeast");

        hBoxes = gobjects(0);
        hTitle = title(ax, "Live LiDAR object scan");
    end

    if ~isempty(hBoxes)
        for k = 1:numel(hBoxes)
            if isgraphics(hBoxes(k))
                delete(hBoxes(k));
            end
        end
    end

    hBoxes = gobjects(0);

    updateScatter(hVeh, plotData.vehCenters);
    updateScatter(hSign, plotData.signCenters);
    updateScatter(hOther, plotData.otherCenters);

    maxBoxes = 30;
    nBoxes = min(numel(plotData.boxes), maxBoxes);
    hBoxes = gobjects(nBoxes * 12, 1);
    hIdx = 0;

    for b = 1:nBoxes
        mm = plotData.boxes{b};
        minPt = mm(1:3);
        maxPt = mm(4:6);

        thisHandles = drawDetectionBox(ax, minPt, maxPt, plotData.boxColors{b}, 1.2);

        for q = 1:numel(thisHandles)
            hIdx = hIdx + 1;
            hBoxes(hIdx) = thisHandles(q);
        end
    end

    hBoxes = hBoxes(1:hIdx);

    nVeh = size(plotData.vehCenters, 1);
    nSign = size(plotData.signCenters, 1);
    nClusters = height(detectionTable);

    set(hTitle, "String", sprintf( ...
        "Live LiDAR scan | frame %d | vehicles %d | signs %d | clusters %d", ...
        frameID, nVeh, nSign, nClusters));

    assignin("base", "latestLiveLidarDetections", detectionTable);

    drawnow limitrate;
end

function updateScatter(h, centers)
    if isempty(centers)
        set(h, "XData", NaN, "YData", NaN, "ZData", NaN);
    else
        set(h, ...
            "XData", centers(:,1), ...
            "YData", centers(:,2), ...
            "ZData", centers(:,3));
    end
end

function h = drawDetectionBox(ax, minPt, maxPt, colorSpec, lineWidth)
    x1 = minPt(1); y1 = minPt(2); z1 = minPt(3);
    x2 = maxPt(1); y2 = maxPt(2); z2 = maxPt(3);

    corners = [
        x1 y1 z1
        x2 y1 z1
        x2 y2 z1
        x1 y2 z1
        x1 y1 z2
        x2 y1 z2
        x2 y2 z2
        x1 y2 z2
    ];

    edges = [
        1 2
        2 3
        3 4
        4 1
        5 6
        6 7
        7 8
        8 5
        1 5
        2 6
        3 7
        4 8
    ];

    h = gobjects(size(edges,1), 1);

    for e = 1:size(edges,1)
        a = corners(edges(e,1), :);
        b = corners(edges(e,2), :);

        h(e) = plot3(ax, ...
            [a(1) b(1)], ...
            [a(2) b(2)], ...
            [a(3) b(3)], ...
            colorSpec, "LineWidth", lineWidth);
    end
end

function [egoPts, sensorPt] = makeDetectionEgoVehicle()
    xFront =  3.2;
    xRear  = -1.3;
    yLeft  =  0.9;
    yRight = -0.9;
    zLow   =  0.0;
    zHigh  =  1.5;

    corners = [
        xFront yLeft  zLow
        xFront yRight zLow
        xRear  yRight zLow
        xRear  yLeft  zLow
        xFront yLeft  zHigh
        xFront yRight zHigh
        xRear  yRight zHigh
        xRear  yLeft  zHigh
    ];

    edges = [
        1 2
        2 3
        3 4
        4 1
        5 6
        6 7
        7 8
        8 5
        1 5
        2 6
        3 7
        4 8
    ];

    egoPts = [];

    for e = 1:size(edges,1)
        a = corners(edges(e,1), :);
        b = corners(edges(e,2), :);

        for k = 0:20
            tLine = k / 20;
            egoPts(end+1,:) = a + tLine * (b - a); 
        end
    end

    sensorPt = [2.7 0 1.6];
end

function distance_m = readPrimaryTrafficStartDistance(testRunFile, fallbackDistance_m)
    distance_m = fallbackDistance_m;

    if ~isfile(testRunFile)
        warning("TestRun file not found: %s. Using fallback ground-truth distance %.3f m.", ...
            testRunFile, fallbackDistance_m);
        return;
    end

    txt = fileread(testRunFile);
    tokens = regexp(txt, "(?m)^Traffic\.0\.StartPos\s*=\s*([-+]?\d*\.?\d+)", ...
        "tokens", "once");

    if isempty(tokens)
        warning("Traffic.0.StartPos was not found in %s. Using fallback ground-truth distance %.3f m.", ...
            testRunFile, fallbackDistance_m);
        return;
    end

    parsedDistance = str2double(tokens{1});
    if isfinite(parsedDistance)
        distance_m = parsedDistance;
    else
        warning("Traffic.0.StartPos could not be parsed in %s. Using fallback ground-truth distance %.3f m.", ...
            testRunFile, fallbackDistance_m);
    end
end

function speed_mps = readPrimaryTrafficLongitudinalSpeed(testRunFile, fallbackSpeed_mps)
    speed_mps = fallbackSpeed_mps;

    if ~isfile(testRunFile)
        warning("TestRun file not found: %s. Using fallback ground-truth speed %.3f m/s.", ...
            testRunFile, fallbackSpeed_mps);
        return;
    end

    txt = fileread(testRunFile);
    tokens = regexp(txt, "(?m)^Traffic\.0\.Man\.0\.LongStep\.0\.Dyn\s*=\s*auto\s+([-+]?\d*\.?\d+)", ...
        "tokens", "once");

    if isempty(tokens)
        warning("Traffic.0.Man.0.LongStep.0.Dyn was not found in %s. Using fallback ground-truth speed %.3f m/s.", ...
            testRunFile, fallbackSpeed_mps);
        return;
    end

    parsedSpeed = str2double(tokens{1});
    if isfinite(parsedSpeed)
        speed_mps = parsedSpeed;
    else
        warning("Traffic.0.Man.0.LongStep.0.Dyn could not be parsed in %s. Using fallback ground-truth speed %.3f m/s.", ...
            testRunFile, fallbackSpeed_mps);
    end
end
function stergeFisiereVechiDataset(outDir)
    modele = [ ...
        "frame_*.mat", ...
        "frame_*_detections.csv", ...
        "cadre_evaluare_lidar.csv", ...
        "rezumat_evaluare_lidar.csv", ...
        "asocieri_evaluare_lidar.csv", ...
        "evaluare_lidar.mat", ...
        "evaluare_lidar.png"];

    for k = 1:numel(modele)
        fisiere = dir(fullfile(outDir, modele(k)));
        for j = 1:numel(fisiere)
            delete(fullfile(fisiere(j).folder, fisiere(j).name));
        end
    end
end


