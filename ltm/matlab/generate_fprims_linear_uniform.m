% Script to generate force primitives
function generate_fprims_linear_uniform(filename)
  num_magnitudes = 2;
  dir = [1.0 0.0 0.0];
  fprims = [dir];
  % Include pushing forces also.
  fprims = [fprims; -fprims]; 
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
