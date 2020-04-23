

function data = com_read(device, data_size)

  if (is_octave)
    data = srl_read(device, data_size);
  else
    data = read(device, data_size, "uint8");
  end

end
