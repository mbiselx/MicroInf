function com_flush(device)

  if (is_octave)
    srl_flush(device);
  else
    flushinput(device);
  end

end
