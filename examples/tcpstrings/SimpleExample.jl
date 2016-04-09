

function sendCmd(cl::TCPSocket, cmd::ASCIIString)
  println(cl, cmd)
  cmd == "QUIT" ?  close(cl) : readline(cl)
end

function getParticles(cl::TCPSocket, lbl::ASCIIString)
  res = sendCmd(cl, "GETPARTICLES $(lbl)")
  rows = split(res[1:(end-1)],';')
  V = readdlm(IOBuffer(rows[1]),',')
  for i in 2:length(rows)
    v = readdlm(IOBuffer(rows[i]),',')
    V = [V;v]
  end
  return V
end


cl = connect(60001)
println("connected")
sendCmd(cl, "INIT")
sendCmd(cl, "ODOMETRY 1 2 10.0 0.0 0.0 0.1 0 0 0.05 0 0.05")
sendCmd(cl, "ODOMETRY 2 3 10.0 0.0 0.0 0.1 0 0 0.05 0 0.05")
Vb = getParticles(cl, "x3");
sendCmd(cl, "BATCHSOLVE")
Va = getParticles(cl, "x3");
@show size(Va)
sendCmd(cl, "LANDMBR 3 4 $(pi/2) 10.0 0.01 0 0.5")
sendCmd(cl, "BATCHSOLVE")
l1 = getParticles(cl, "l1");
@show size(l1)


sendCmd(cl, "QUIT")
close(cl)
