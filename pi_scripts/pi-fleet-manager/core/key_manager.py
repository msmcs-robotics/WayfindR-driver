import os
import subprocess
from pathlib import Path
from typing import Optional, List
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.backends import default_backend

class KeyManager:
    """Manage SSH keys"""
    
    def __init__(self):
        self.ssh_dir = Path.home() / ".ssh"
        self.ssh_dir.mkdir(mode=0o700, exist_ok=True)
    
    def generate_key(self, key_name: str, key_size: int = 4096) -> dict:
        """
        Generate new SSH key pair
        
        Args:
            key_name: Name for the key (will be saved as ~/.ssh/{key_name})
            key_size: RSA key size in bits
        
        Returns:
            dict with private_key_path and public_key_path
        """
        private_key_path = self.ssh_dir / key_name
        public_key_path = self.ssh_dir / f"{key_name}.pub"
        
        # Generate RSA key
        key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=key_size,
            backend=default_backend()
        )
        
        # Write private key
        private_pem = key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.OpenSSH,
            encryption_algorithm=serialization.NoEncryption()
        )
        private_key_path.write_bytes(private_pem)
        private_key_path.chmod(0o600)
        
        # Write public key
        public_key = key.public_key()
        public_ssh = public_key.public_bytes(
            encoding=serialization.Encoding.OpenSSH,
            format=serialization.PublicFormat.OpenSSH
        )
        public_key_path.write_text(public_ssh.decode('utf-8'))
        
        return {
            "private_key": str(private_key_path),
            "public_key": str(public_key_path),
            "name": key_name
        }
    
    def list_keys(self) -> List[dict]:
        """List all SSH keys in ~/.ssh"""
        keys = []
        for key_file in self.ssh_dir.glob("*"):
            if key_file.is_file() and not key_file.name.endswith('.pub'):
                # Check if it's a private key
                try:
                    with open(key_file, 'r') as f:
                        content = f.read()
                        if 'PRIVATE KEY' in content:
                            pub_key = self.ssh_dir / f"{key_file.name}.pub"
                            keys.append({
                                "name": key_file.name,
                                "private_key": str(key_file),
                                "public_key": str(pub_key) if pub_key.exists() else None
                            })
                except:
                    pass
        return keys
    
    def get_public_key(self, private_key_path: str) -> Optional[str]:
        """Get public key content from private key path"""
        pub_key_path = f"{private_key_path}.pub"
        if Path(pub_key_path).exists():
            return Path(pub_key_path).read_text().strip()
        return None
    
    def ensure_agent_running(self) -> bool:
        """Ensure SSH agent is running"""
        try:
            # Check if agent is running
            result = subprocess.run(
                ['ssh-add', '-l'],
                capture_output=True,
                text=True
            )
            # If exit code is 0 or 1, agent is running (1 means no keys loaded)
            return result.returncode in [0, 1]
        except:
            return False
    
    def add_key_to_agent(self, key_path: str) -> bool:
        """Add key to SSH agent"""
        try:
            subprocess.run(
                ['ssh-add', key_path],
                check=True,
                capture_output=True
            )
            return True
        except:
            return False
    
    def setup_shell_rc(self) -> bool:
        """Add SSH agent initialization to shell RC files"""
        agent_code = '''
# SSH Agent auto-start
if [ -z "$SSH_AUTH_SOCK" ]; then
    eval "$(ssh-agent -s)" > /dev/null
fi
'''
        
        rc_files = [
            Path.home() / ".bashrc",
            Path.home() / ".zshrc"
        ]
        
        success = False
        for rc_file in rc_files:
            if rc_file.exists():
                content = rc_file.read_text()
                if "ssh-agent -s" not in content:
                    with open(rc_file, 'a') as f:
                        f.write(agent_code)
                    success = True
        
        return success
    
    def copy_key_to_host(self, key_path: str, host: str, port: int, 
                        username: str, password: Optional[str] = None) -> bool:
        """
        Copy SSH key to remote host (ssh-copy-id equivalent)
        
        Args:
            key_path: Path to private key
            host: Target host
            port: SSH port
            username: Username
            password: Password for initial connection (if needed)
        """
        import paramiko
        
        pub_key_path = f"{key_path}.pub"
        if not Path(pub_key_path).exists():
            return False
        
        pub_key = Path(pub_key_path).read_text().strip()
        
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            # Connect with password or key
            if password:
                client.connect(host, port=port, username=username, password=password)
            else:
                client.connect(host, port=port, username=username, key_filename=key_path)
            
            # Ensure .ssh directory exists
            commands = [
                'mkdir -p ~/.ssh',
                'chmod 700 ~/.ssh',
                f'echo "{pub_key}" >> ~/.ssh/authorized_keys',
                'chmod 600 ~/.ssh/authorized_keys'
            ]
            
            for cmd in commands:
                stdin, stdout, stderr = client.exec_command(cmd)
                stdout.channel.recv_exit_status()
            
            client.close()
            return True
            
        except Exception as e:
            print(f"Error copying key: {e}")
            return False