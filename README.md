#Monitoring Ketinggian Air dan PH Level berbasis ESP32

Program Arduino ini dirancang untuk ESP32 guna memantau dan mengunggah data sensor ke broker MQTT, dengan kemampuan menyimpan data secara offline jika koneksi terputus.

---

## **Alur Kerja Program**

Berikut adalah alur kerja utama dari program ini:

### **1. Inisialisasi (`setup()`)**

* **Serial Communication**: Memulai komunikasi serial untuk debugging.
* **LED**: Mengatur pin LED bawaan ESP32 sebagai *output* dan menyalakannya.
* **pH Sensor**: Mengatur resolusi ADC untuk sensor pH.
* **SPIFFS (Flash Memory)**: Memulai sistem berkas SPIFFS untuk menyimpan data *offline*. Jika gagal, program akan berhenti.
* **I2C & RTC DS3231**: Menginisialisasi komunikasi I2C pada pin khusus (SDA 21, SCL 22) dan memulai modul RTC (Real-Time Clock). Jika RTC tidak ditemukan, program akan berhenti.
* **INA219 Sensor**: Menginisialisasi sensor INA219 (pengukur tegangan dan arus). Jika sensor tidak ditemukan, program akan berhenti.
* **HX711 Scale**: Menginisialisasi modul HX711 untuk sensor berat, mengatur *scale*, dan melakukan *tare* (mengatur ulang ke nol).
* **Wi-Fi**: Mengatur ESP32 untuk secara otomatis mencoba menyambung kembali ke Wi-Fi dan memulai koneksi ke jaringan yang ditentukan (`ssid`, `password`).
* **MQTT Client**: Mengatur server dan port broker MQTT (EMQX) serta menginisialisasi klien MQTT dengan kredensial SSL (CA certificate).
* **OTA (Over-The-Air) Update**: Mempersiapkan fungsi *update firmware* secara nirkabel dengan mengatur kata sandi dan *callback* untuk berbagai status *update*.

---

### **2. Loop Utama (`loop()`)**

* **Penanganan OTA**: Jika Wi-Fi terhubung, program akan menginisialisasi dan menangani *update* OTA. Ini memungkinkan *firmware* diperbarui dari jarak jauh melalui jaringan.
* **Pengecekan Koneksi Wi-Fi**:
    * Jika Wi-Fi terputus, LED akan menyala dan program akan mencoba menyambung kembali (`reconnectWiFi()`).
    * Jika `reconnectWiFi()` gagal, program akan masuk ke `offlineMode()`.
* **Sinkronisasi Waktu**: Jika Wi-Fi terhubung, program akan mencoba menyinkronkan waktu dengan server NTP (`syncTime()`). Jika gagal, program akan terus berjalan.
* **Pengecekan Koneksi MQTT**:
    * Jika MQTT terputus, program akan mencoba menyambung kembali (`reconnectMQTT()`).
    * Jika `reconnectMQTT()` gagal, program akan menunda eksekusi sejenak dan mencoba lagi di *loop* berikutnya.
* **Operasi Online**: Jika MQTT terhubung:
    * Klien MQTT akan menjalankan *loop* untuk memproses pesan masuk dan keluar (`client.loop()`).
    * Data sensor (pH, level air, arus) akan dibaca dan dipublikasikan ke broker MQTT (`publishOnline()`).
    * Ada jeda 3 detik sebelum *loop* berikutnya.

---

### **3. Fungsi Pembantu**

* **`setupOTA()`**: Mengatur berbagai *callback* untuk proses *Over-The-Air (OTA)* *update*, seperti saat *update* dimulai, selesai, progres, dan penanganan *error*.
* **`syncTime()`**:
    * Mengatur konfigurasi waktu untuk NTP.
    * Mencoba menyinkronkan waktu dengan server NTP yang ditentukan.
    * Jika berhasil, waktu RTC akan disesuaikan dengan waktu NTP. LED akan berkedip selama proses sinkronisasi.
* **`reconnectWiFi()`**:
    * Mencoba memutuskan dan menyambungkan kembali ke jaringan Wi-Fi.
    * Ada penundaan minimum antar percobaan dan *timeout* untuk koneksi.
    * Jika berhasil, alamat IP akan dicetak, dan *flag* OTA akan direset untuk inisialisasi ulang.
* **`sendOfflineData()`**:
    * Membaca data yang disimpan di file `/data.txt` di SPIFFS.
    * Setiap baris data akan dipublikasikan ke topik MQTT `kelA1/offline`.
    * Setelah semua data berhasil dikirim, file `/data.txt` akan dihapus. Ini memastikan data *offline* tidak dikirim berulang kali.
* **`reconnectMQTT()`**:
    * Mengatur sertifikat CA untuk koneksi MQTT yang aman.
    * Mencoba menghubungkan klien MQTT ke broker menggunakan kredensial yang ditentukan.
    * Jika berhasil, LED akan mati (menandakan koneksi), status "online" akan dipublikasikan ke `kelA1/status/1101`, dan data *offline* yang tersimpan akan dikirim.
* **`readPH()`**:
    * Membaca nilai analog dari sensor pH.
    * Mengubah nilai analog menjadi tegangan.
    * Melakukan kalibrasi linier berdasarkan titik kalibrasi yang ditentukan (`PH4_VOLTAGE`, `PH7_VOLTAGE`) untuk mendapatkan nilai pH.
    * Mencetak nilai analog, tegangan, dan pH ke Serial Monitor.
* **`publishOnline()`**:
    * Membaca nilai dari sensor pH, sensor level air (HX711), dan sensor arus/tegangan (INA219).
    * Membuat objek JSON yang berisi data sensor (`pressure` (nilai mentah HX711), `distance` (level air dalam cm), `ph`, `Arus`).
    * Mengubah objek JSON menjadi *string* dan mempublikasikannya ke topik MQTT `kelA1/online`.
* **`offlineMode()`**:
    * Mode ini diaktifkan ketika Wi-Fi terputus.
    * Secara periodik membaca data sensor (pH, level air, arus, tegangan, dan waktu dari RTC).
    * Membuat *string* JSON dari data sensor dan menyimpannya ke file `/data.txt` di SPIFFS (`saveDataToSPIFFS()`).
    * Setiap satu menit, program akan mencoba keluar dari mode *offline* dengan mencoba menyambung kembali ke Wi-Fi.
    * LED akan berkedip saat mode *offline* aktif.
* **`saveDataToSPIFFS(String data)`**: Menambahkan data yang diberikan ke file `/data.txt` di SPIFFS.
* **`interp(long x0, float y0, long x1, float y1, long x)`**: Fungsi pembantu untuk interpolasi linier, digunakan dalam kalibrasi level air.
* **`rawToCm(long raw)`**: Mengkonversi nilai mentah dari sensor HX711 menjadi tinggi level air dalam sentimeter menggunakan interpolasi berdasarkan titik kalibrasi yang diketahui. Ini juga menangani ekstrapolasi di luar titik kalibrasi.

---
