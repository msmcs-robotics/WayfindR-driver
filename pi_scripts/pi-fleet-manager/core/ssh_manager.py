import paramiko
import asyncio
from typing import Optional, Dict
import socket

class SSHManager:
    """Manage SSH connections to hosts"""
    
    def __init__(self):
        self.connections = {}
    
    def connect(self, host: str, port: int, username: str, 
                key_path: str, jumphost: Optional[Dict] = None) -> paramiko.SSHClient:
        """
        Create SSH connection to host
        
        Args:
            host: Target host IP/hostname
            port: SSH port
            username: Username to connect as
            key_path: Path to SSH private key
            jumphost: Optional dict with jumphost config
        """
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            if jumphost:
                # Connect through jumphost
                jump_client = paramiko.SSHClient()
                jump_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                jump_client.connect(
                    hostname=jumphost['host'],
                    port=jumphost.get('port', 22),
                    username=jumphost['username'],
                    key_filename=key_path,
                    timeout=10
                )
                
                # Create channel through jumphost
                jump_transport = jump_client.get_transport()
                dest_addr = (host, port)
                local_addr = ('127.0.0.1', 0)
                channel = jump_transport.open_channel("direct-tcpip", dest_addr, local_addr)
                
                # Connect to final host through channel
                client.connect(
                    hostname=host,
                    port=port,
                    username=username,
                    key_filename=key_path,
                    sock=channel,
                    timeout=10
                )
            else:
                # Direct connection
                client.connect(
                    hostname=host,
                    port=port,
                    username=username,
                    key_filename=key_path,
                    timeout=10
                )
            
            return client
            
        except Exception as e:
            raise Exception(f"Failed to connect to {host}: {str(e)}")
    
    def execute_command(self, client: paramiko.SSHClient, command: str) -> Dict[str, str]:
        """Execute command on remote host"""
        try:
            stdin, stdout, stderr = client.exec_command(command)
            return {
                "stdout": stdout.read().decode('utf-8'),
                "stderr": stderr.read().decode('utf-8'),
                "exit_code": stdout.channel.recv_exit_status()
            }
        except Exception as e:
            return {
                "stdout": "",
                "stderr": str(e),
                "exit_code": 1
            }
    
    def close(self, client: paramiko.SSHClient):
        """Close SSH connection"""
        if client:
            client.close()
    
    def test_connection(self, host: str, port: int, username: str, 
                       key_path: str, jumphost: Optional[Dict] = None) -> bool:
        """Test if connection is possible"""
        try:
            client = self.connect(host, port, username, key_path, jumphost)
            self.close(client)
            return True
        except:
            return False
    
    def copy_file(self, client: paramiko.SSHClient, local_path: str, 
                  remote_path: str) -> bool:
        """Copy file to remote host using SFTP"""
        try:
            sftp = client.open_sftp()
            sftp.put(local_path, remote_path)
            sftp.close()
            return True
        except Exception as e:
            print(f"Error copying file: {e}")
            return False
    
    def copy_directory(self, client: paramiko.SSHClient, local_dir: str, 
                       remote_dir: str) -> bool:
        """Recursively copy directory to remote host"""
        try:
            sftp = client.open_sftp()
            
            # Ensure remote directory exists
            try:
                sftp.stat(remote_dir)
            except:
                sftp.mkdir(remote_dir)
            
            from pathlib import Path
            for item in Path(local_dir).rglob('*'):
                if item.is_file():
                    remote_file = str(Path(remote_dir) / item.relative_to(local_dir))
                    remote_file = remote_file.replace('\\', '/')
                    
                    # Create remote subdirectories if needed
                    remote_subdir = '/'.join(remote_file.split('/')[:-1])
                    try:
                        sftp.stat(remote_subdir)
                    except:
                        # Create directory structure
                        parts = remote_subdir.split('/')
                        current = ''
                        for part in parts:
                            if part:
                                current += '/' + part
                                try:
                                    sftp.stat(current)
                                except:
                                    sftp.mkdir(current)
                    
                    sftp.put(str(item), remote_file)
            
            sftp.close()
            return True
        except Exception as e:
            print(f"Error copying directory: {e}")
            return False