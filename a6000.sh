ssh -N \
  -L 49100:127.0.0.1:49100 \
  -p 6519 \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=10 \
  -o ServerAliveCountMax=3 \
  pnudtn10@164.125.19.141
