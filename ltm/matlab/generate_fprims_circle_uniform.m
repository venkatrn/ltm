% Script to generate force primitives
function generate_fprims_circle_uniform(filename)
  %TODO: Allow arbitrary circles in 3D. Not just parallel to z-axis.
  num_points_on_circle = 40;
  num_magnitudes = 1;
  angles = linspace(-pi, pi, num_points_on_circle + 1)';
  angles(end) = [];
  fprims = [cos(angles) sin(angles) zeros(numel(angles),1)];
  for i = 2:num_magnitudes
    fprims = [fprims; i*fprims];
  end
  PrintToFile(filename, fprims);
end

function PrintToFile(filename, fprims)
  fid = fopen(filename, 'w+');
  points = [[0:size(fprims,1)-1]' fprims];
  fprintf(fid, '%s\n', sprintf('primitives: %d', size(fprims, 1)));
  fclose(fid);
  dlmwrite(filename, fprims, 'delimiter', ' ','-append', 'precision', '%.3f');
end
