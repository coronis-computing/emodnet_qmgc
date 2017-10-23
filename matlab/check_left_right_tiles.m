function check_left_right_tiles( TileOffLeft, TileOffRight ) 

[TileLeftVert, TileLeftFaces] = Off2Mat( TileOffLeft ) ;
[TileRightVert, TileRightFaces] = Off2Mat( TileOffRight ) ;

% Shift the vertices of the right tile
maxVal = 32767 ;

TileRightVert( :, 1 ) = TileRightVert( :, 1 ) + maxVal - 1 ; 

figure ;
hold on ;
trimesh( TileLeftFaces, TileLeftVert( :, 1 ), TileLeftVert( :, 2 ), TileLeftVert( :, 3 ) ) ;
trimesh( TileRightFaces, TileRightVert( :, 1 ), TileRightVert( :, 2 ), TileRightVert( :, 3 ) ) ;
axis equal ;
hold off ;
