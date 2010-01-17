function [t, pos, pot, enc, press] = load_brake(filename)
  load(filename);
  t = brake(:,1);
  pos = brake(:,4);
  pot = brake(:,5);
  enc = brake(:,6);
  press = brake(:,7);
