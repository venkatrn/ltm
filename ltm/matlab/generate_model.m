
%% Script to generate d-models 

function [points, edges] = generate_model(filename)
  num_points = 0;
  num_edges = 0;
  points = [];
  edges = [];
  x_lims = [1 2];
  y_lims = [-0.75 0.75];
  z_lims = [0 1.0];
  res = 0.2;
  p = Cube(x_lims, y_lims, z_lims, res, res, res);
  e = ComputeEdges(p);
  points = [points; p];
  edges = [edges; e];
  num_points = num_points + size(points,1);
  num_edges = num_edges + size(edges,1);
  x_lims = [0.85 0.85];
  y_lims = [-0.75 0.75];
  z_lims = [0.6 1.0];
  y_res = 0.2;
  x_res = 0.1;
  z_res = 0.2;
  p = Cube(x_lims, y_lims, z_lims, x_res, y_res, z_res);
  e = ComputeEdges(p);
  e = e + num_points;
  points = [points; p];
  edges = [edges; e];
  cross_edges = ComputeCrossEdges(p, points(1:num_points,:));
  PrintToFile(filename, points, edges, cross_edges);
end


function points = Cube(x_lims, y_lims, z_lims, x_res, y_res, z_res)
  %xv = linspace(x_lims(1), x_lims(2), uint8((x_lims(2)-x_lims(1))/x_res + 0.5));
  %yv = linspace(y_lims(1), y_lims(2), uint8((y_lims(2)-y_lims(1))/y_res + 0.5));
  %zv = linspace(z_lims(1), z_lims(2), uint8((z_lims(2)-z_lims(1))/z_res + 0.5));
  xv = x_lims(1):x_res:x_lims(2);
  yv = y_lims(1):y_res:y_lims(2);
  zv = z_lims(1):z_res:z_lims(2);
  [X,Y,Z] = meshgrid(xv, yv, zv);
  points = [X(:) Y(:) Z(:)];
end

function edges = ComputeEdges(points)
  [edges, d] = knnsearch(points, points, 'K', 7);
  edges(:,1) = [];
end

function edges = ComputeCrossEdges(p1, p2)
  [edges, d] = knnsearch(p2, p1, 'K', 1);
  edges = [edges zeros(size(edges,1), 6)];
end

function PrintToFile(filename, points, edges, cross_edges)
  fid = fopen(filename, 'w+');
  points = [[0:size(points,1)-1]' points zeros(size(points,1), 3) ones(size(points,1),1)];
  fprintf(fid, '%s\n', sprintf('points: %d', size(points, 1)));
  fprintf(fid, '%s\n', sprintf('edges: %d', size(edges, 1)*size(edges, 2) + size(cross_edges,1)));
  fclose(fid);
  dlmwrite(filename, points, 'delimiter', ' ','-append');
  fid = fopen(filename, 'a+');
  for ii = 1:size(edges,1)
    for jj = 1:size(edges,2)
      if (edges(ii, jj) == 0)
        continue;
      end
      fprintf(fid, '%d %d %d %f %f %f %f\n', ii-1, edges(ii, jj)-1, 0, 0, 0, 0, 1);
    end
  end
  for ii = 1:size(cross_edges,1)
      fprintf(fid, '%d %d %d %f %f %f %f\n', ii-1+size(points,1)-size(cross_edges,1), cross_edges(ii, 1)-1, 1, 1.0, 0, 0, 1);
  end
end
