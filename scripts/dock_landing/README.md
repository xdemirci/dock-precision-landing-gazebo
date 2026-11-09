Başlatmak için `main.py`
Görevi başlatmak için SPACE, sabit yaw(kuzey) için "y", çıkmak için "q" bas.
| Dosya | Görev |
|---|---|
| config.py | tüm argümanlar (tek yer) |
| camera.py | GStreamer, arka plan thread, hep en son kare |
| detector.py | ArUco tespiti + geometri/ID/ardisik-kare filtreleri |
| pose.py | kamera modeli (+kalibrasyon), PnP, aci -> NED donusumleri |
| estimator.py | PosVelEKF portu + N/E hedef filtresi |
| vehicle.py | MAVLink durum/komut (bloklamaz) |
| mission.py | iniş faz makinesi (HİZALA -> ALÇAL -> SON İNİŞ -> TEMAS -> DISARM) |
| controller.py | P kontrol + hiz/ivme siniri + yaw hizalama |
| height.py | marker yuksekligini PnP'den ogrenir, marker kaybolunca EKF z ile devam |
| hud.py | ekran paneli |
| main.py | hepsini bağlayan ince döngü |

Yükseklik (h) = marker düzlemine yükseklik. Kaynak: --h-src ekf (z - --dock-h) | rng | pnp.
Yakında (h<3 m) marker kaybolursa kare `lost_HHMMSS.png` olarak kaydedilir.
<img width="1127" height="803" alt="Screenshot from 2026-11-09 20-29-46" src="https://github.com/user-attachments/assets/65bd8c35-4738-4c03-9c91-2e25ec5ae262" />
