
I want to make a simple fast api app that will be network facing that acts as a forwarding app for ollama. so basically i have a HPC cluster, a local ubuntu server, then a raspberry pi. the hpc cluster runs the LLMs an AI models, becuase it has more computing resources, but i need to VPN to connect to it. I don't want to use my credentials from the pi, so i would rather vpn from the ubuntu server and then make a fast api python app that simply is network facing for the rpi or other clients to interact with ollama on port 11434 but just not on localhost, but on an ip address. how can i build a simple app to do this? I already have port forwarding and ssh access and vpn setup on ubuntu server to be able to access ollama on localhost:11434 in context from the ubuntu server, how would i make it available to the network?




