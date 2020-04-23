

function data = com_read(device, data_size)

  if (is_octave)
    data = srl_read(device, data_size);
  else
    readasync(device, data_size);
    data = fscanf(s);
  end

end
