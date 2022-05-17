function stats = loadStats(baseDir, methods, baseStatsFileName)
%LOADSTATS Loads the stats files generated by the dev-tests/compute_statistics exe

if nargin < 3
    baseStatsFileName = 'out_stats_zoom_';
end

numMethods = numel(methods);
stats(numMethods).method = '';

for i = 1:numMethods
    method = methods{i}
    stats(i).method = method;
    
    % Check the number of zooms in that method
    dirCnt = dir([baseDir '/' method '/' baseStatsFileName '*.m']);
    numZooms = numel(dirCnt);
    
    stats(i).zooms(numZooms).zoom = 0;
    stats(i).zooms(numZooms).meanNumVertPerTile = [];
    stats(i).zooms(numZooms).hausdorffDistRasterToTinPerTile = [];
    stats(i).zooms(numZooms).hausdorffDistTinToRasterPerTile = [];
    stats(i).zooms(numZooms).maxHeightError = [];
    stats(i).zooms(numZooms).maxHeightErrorPerTile = [];
    stats(i).zooms(numZooms).meanHeightError = [];
    stats(i).zooms(numZooms).meanHeightErrorPerTile = [];
    stats(i).zooms(numZooms).meanNumTriangles = [];
    stats(i).zooms(numZooms).meanNumTrianglesPerTile = [];
    stats(i).zooms(numZooms).meanNumVert = [];
    stats(i).zooms(numZooms).meanNumVertPerTile = [];
    stats(i).zooms(numZooms).meanSymmetricHausdorffDistance = [];
    stats(i).zooms(numZooms).numTrianglesRasterTile = [];
    stats(i).zooms(numZooms).numVertRasterTile = [];
    stats(i).zooms(numZooms).symmetricHausdorffDistancePerTile = [];
    stats(i).zooms(numZooms).tileXY = [];
    
    for z = 1:numel(dirCnt)
        % Load the statistics corresponding to that zoom
        statsFilePath = fullfile(baseDir, method, dirCnt(z).name);
        run(statsFilePath);
    
        % zoom number
        zoomStr = erase(erase(dirCnt(z).name, baseStatsFileName), '.m');
        zoom = str2double(zoomStr);
        
        stats(i).zooms(z).zoom = zoom;
        stats(i).zooms(z).meanNumVertPerTile = meanNumVertPerTile;
        stats(i).zooms(z).hausdorffDistRasterToTinPerTile = hausdorffDistRasterToTinPerTile;
        stats(i).zooms(z).hausdorffDistTinToRasterPerTile = hausdorffDistTinToRasterPerTile;
        stats(i).zooms(z).maxHeightError = maxHeightError;
        stats(i).zooms(z).maxHeightErrorPerTile = maxHeightErrorPerTile;
        stats(i).zooms(z).meanHeightError = meanHeightError;
        stats(i).zooms(z).meanHeightErrorPerTile = meanHeightErrorPerTile;
        stats(i).zooms(z).meanNumTriangles = meanNumTriangles;
        stats(i).zooms(z).meanNumTrianglesPerTile = meanNumTrianglesPerTile;
        stats(i).zooms(z).meanNumVert = meanNumVert;
        stats(i).zooms(z).meanNumVertPerTile = meanNumVertPerTile;
        stats(i).zooms(z).meanSymmetricHausdorffDistance = meanSymmetricHausdorffDistance;
        stats(i).zooms(z).numTrianglesRasterTile = numTrianglesRasterTile;
        stats(i).zooms(z).numVertRasterTile = numVertRasterTile;
        stats(i).zooms(z).symmetricHausdorffDistancePerTile = symmetricHausdorffDistancePerTile;
        stats(i).zooms(z).tileXY = tileXY;
        
        clear zoom meanNumVertPerTile hausdorffDistRasterToTinPerTile hausdorffDistTinToRasterPerTile maxHeightError maxHeightErrorPerTile meanHeightError meanHeightErrorPerTile meanNumTriangles  meanNumTrianglesPerTile meanNumVert meanNumVertPerTile meanSymmetricHausdorffDistance numTrianglesRasterTile numVertRasterTile symmetricHausdorffDistancePerTile tileXY
    end    
end

