On your host:

Install vscode with c++ extensions package
Update .vscode/launch.json and c_cpp_properties.json with proper paths (change paths as appropriate)

sudo apt-get install gdb-multiarch

In your devel directory:
  sshfs ubuntu@192.168.5.40:/ ./rover

ssh -L9091:localhost:9091 ubuntu@ROVER_IP
gdbserver :9091 ./bin/mecanum_drive_controller

References:

https://medium.com/@spe_/debugging-c-c-programs-remotely-using-visual-studio-code-and-gdbserver-559d3434fb78
https://medium.com/@karel.l.vermeiren/cross-architecture-remote-debugging-using-gdb-with-visual-studio-code-vscode-on-linux-c0572794b4ef
https://nnfw.readthedocs.io/en/stable/howto/how-to-remote-debugging-with-visual-studio-code.html
