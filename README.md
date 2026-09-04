# Tactical UAV Pathfinding — Autonomy Simulation Prototype

*[Read this in English](README.en.md)*

Unity tabanlı, taktik bir İHA'nın (UAV) dinamik tehditler ve belirsiz sensör
verisi altında rota planlaması ve tehdit kaçınması yapmasını simüle eden bir
otonomi prototipi. Proje, Selçuk Üniversitesi Teknokent bünyesindeki staj
kapsamında geliştirilmiştir.

## Projenin Amacı

Sistem, tek bir "en kısa yolu bul" algoritmasından ibaret değildir. Amaç,
gerçek bir otonom sistemde bulunan uçtan uca zinciri simüle etmektir:

```
Sensor → State Estimation → Tracking → Threat Assessment → Replanning → PathFollower → Mission/Telemetry
```

Bu zincirin her halkası ayrı ayrı geliştirilmiş ve test edilmiştir.

## Mimari

| Katman | Sorumluluk | Ana dosyalar |
|---|---|---|
| **Sensors** | GPS, IMU, Barometre, LiDAR, Radar simülasyonu; Gaussian gürültü, sensör arıza enjeksiyonu | `Assets/Scripts/Sensors/` |
| **State Estimation** | Extended Kalman Filter ile pozisyon/hız/yön tahmini, belirsizlik (covariance) takibi | `Assets/Scripts/StateEstimation/` |
| **Tracking** | Çoklu hedef takibi, sensör verisi ilişkilendirme (data association), track yaşam döngüsü | `Assets/Scripts/Tracking/` |
| **Threat Assessment** | TTC/CPA hesaplama, belirsizlik-farkında tehdit skorlama, çoklu tehdit önceliklendirme | `ThreatAssessment.cs` |
| **Pathfinding & Replanning** | A* tabanlı rota planlama, Velocity Obstacle kaçınma (3 aşamalı: hız düşürme → dikey kaçınma → uzamsal replan) | `Pathfinding.cs`, `ReplanningController.cs` |
| **Mission** | Görev durumu, skor hesaplama, olay günlüğü | `MissionManager.cs`, `MissionScore.cs`, `MissionEventLogger.cs` |
| **Diagnostics** | Sahne içi 3D görselleştirme (LiDAR/Radar/tehdit/EKF belirsizliği) | `Assets/Scripts/Diagnostics/` |

## Test ve Doğrulama

Proje iki katmanlı bir test yapısı kullanır:

- **EditMode testleri** (`Assets/Tests/EditMode/`, 46 dosya) — algoritma
  birimlerinin izole doğrulanması: pathfinding, EKF, sensör füzyonu, tehdit
  değerlendirme, çoklu tehdit önceliklendirme, GPS dayanıklılığı.
- **PlayMode testleri** (`Assets/Tests/PlayMode/`, 3 dosya) — sahne içinde
  çalışan runtime senaryoları.

Ayrıca `Assets/Editor/BenchmarkSuiteRunner.cs` ve `BenchmarkReporter.cs`
üzerinden çalışan bir benchmark altyapısı ve `Assets/Scenarios/` altında
tanımlı senaryo varlıkları (dense obstacles, dynamic threats, long range,
3D vertical climb vb.) bulunmaktadır.

**Not — kapsam ve sınırlamalar hakkında dürüst bir açıklama:**
Benchmark senaryolarının bir kısmı gerçek uçtan uca (end-to-end) production
senaryolarıdır; bir kısmı ise GPS/belirsizlik matematiğinin veya çoklu tehdit
mantığının **kontrollü enjeksiyon** ile izole test edilmesidir (örn. gerçek
zamanlı sürekli bir GPS kesintisi yerine, EKF'e doğrudan covariance/hata
enjekte edilerek davranış doğrulanmıştır). Bu ayrım kasıtlı olarak
gizlenmemiştir; hangi senaryonun hangi kategoriye girdiği test dosyası
isimlerinden ve final raporda anlaşılabilir.

## Kapsam Dışı Kalanlar

- Gerçek fiziksel UAV donanımı üzerinde test (staj kapsamı dışında)
- Gerçek GPS/IMU/LiDAR/Radar donanımı — tüm sensörler simüle edilmiştir
- Uzun süreli, sürekli gerçek zamanlı GPS outage benchmark'ı
- Gerçek zamanlı performans/CPU/frame-budget deployment metrikleri
- ROS 2 / MAVLink / PX4 gibi gerçek otonomi stack'lerine köprü

Bu kalemler bilinçli olarak "future work" (gelecek çalışma) olarak
bırakılmıştır; mimari bu yönde genişlemeye uygun şekilde katmanlandırılmıştır
(örn. `ISensor` arayüzü, sensör kaynağının gerçek donanımla değiştirilmesine
izin verir).

## Proje Yapısı

```
Assets/
  Scripts/           Ana otonomi kodu (sensör, state estimation, tracking, planning)
    Sensors/
    StateEstimation/
    Tracking/
    Diagnostics/
  Scenarios/          Benchmark ve test senaryosu ScriptableObject varlıkları
  Editor/              Benchmark koşucusu (BenchmarkSuiteRunner)
  Tests/
    EditMode/          Birim ve entegrasyon testleri
    PlayMode/          Runtime/sahne testleri
```

## Nasıl Çalıştırılır

1. Unity Editor ile projeyi açın (Unity 6 / son LTS önerilir).
2. Test çalıştırmak için: `Window → General → Test Runner`, EditMode ve
   PlayMode sekmelerinden ilgili testleri seçip çalıştırın.
3. Benchmark koşusu için: `Assets/Editor/BenchmarkSuiteRunner.cs` üzerinden
   tanımlı Editor menü komutunu kullanın; sonuçlar `BenchmarkReporter`
   aracılığıyla raporlanır.
4. Senaryo denemek için `Assets/Scenarios/` altındaki `.asset` dosyalarından
   birini seçip ilgili sahneye atayın.

## Geliştirme Notu

Bu proje geliştirme sürecinde AI destekli araçlar (kod üretimi ve test
yazımı dahil) kullanılmıştır. Mimari kararlar ve sistem tasarımı
geliştirici tarafından yönlendirilmiş; üretilen kod ve testler gözden
geçirilerek entegre edilmiştir.
