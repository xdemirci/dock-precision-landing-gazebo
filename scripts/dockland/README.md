# dockland: ArUco ile dock ortalama + inis (kendi PosVel KF)

    cd dockland
    python3 selftest.py                       # donanimsiz test
    python3 main.py --conn udp:127.0.0.1:14550      # rangefinder'siz (PnP ile yukseklik)

Tuslar: SPACE start/pause, y yaw hizalama, q cikis.

| dosya | gorev |
|---|---|
| config.py | tum argumanlar (tek yer) |
| camera.py | GStreamer, arka plan thread, hep en son kare |
| detector.py | ArUco tespiti + geometri/ID/ardisik-kare filtreleri |
| pose.py | kamera modeli (+kalibrasyon), PnP, aci -> NED donusumleri |
| estimator.py | PosVelEKF portu + N/E hedef filtresi |
| vehicle.py | MAVLink durum/komut (bloklamaz) |
| mission.py | inis faz makinesi (HIZALA -> ALCAL -> SON INIS -> TEMAS -> DISARM) |
| controller.py | P kontrol + hiz/ivme siniri + yaw hizalama |
| height.py | marker yuksekligini PnP'den ogrenir, marker kaybolunca EKF z ile devam |
| hud.py | ekran paneli |
| main.py | hepsini baglayan ince dongu |

Yukseklik (h) = marker duzlemine yukseklik. Kaynak: --h-src ekf (z - --dock-h) | rng | pnp.
ArduPilot precland'li surum ayri dosyada: precland_ardu.py
