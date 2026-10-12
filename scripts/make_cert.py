"""WebXR / 実機ブラウザ用の自己署名証明書を作る (data/certs/)。

WebXR の没入セッションは HTTPS (secure context) が必須。自己署名なので、ヘッドセット/iPad のブラウザで
初回に警告が出る (詳細 → 続行)。SAN に localhost・この PC の LAN IP・ホスト名を入れるので、LAN 内の他端末から
https://<PC の IP>:8443 で開ける。
"""
from __future__ import annotations

import datetime
import ipaddress
import socket
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "certs"


def lan_ips() -> list[str]:
    ips = {"127.0.0.1"}
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))   # 実際には送信されない。経路選択で既定 NIC の IP を得る
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def main(force: bool = False) -> tuple[Path, Path]:
    OUT.mkdir(parents=True, exist_ok=True)
    key_p, crt_p = OUT / "key.pem", OUT / "cert.pem"
    if key_p.exists() and crt_p.exists() and not force:
        return key_p, crt_p
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "vrso101-local")])
    san = [x509.DNSName("localhost"), x509.DNSName(socket.gethostname())] + [x509.IPAddress(ipaddress.ip_address(i)) for i in lan_ips()]
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=825))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .sign(key, hashes.SHA256()))
    key_p.write_bytes(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                                        serialization.NoEncryption()))
    crt_p.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    print(f"証明書を作成: {crt_p} (SAN: {', '.join(lan_ips())})")
    return key_p, crt_p


if __name__ == "__main__":
    main(force="--force" in sys.argv)
    print("LAN URL: " + " / ".join(f"https://{ip}:8443" for ip in lan_ips() if ip != "127.0.0.1"))
