# За прилагане протокола на ELL за работа с цифрови вх/изх. по uart върху процесори ESP32
Подготовка за работа с устройства по проекта Controller с ESP32-S3.  
Цели:
- [x] пускане на slave устройство (не ни е нужно засега, но е за тестове на разработката);
- [x] пускане на master устройство;
# Постигнати резултати
- успешно пуснати master и slave устройства;
- 16 байта входове и 16 байта изходи се обработват по протокола за ~8.4mS;
    - при подобренията заради работа и с WiFi времето се увеличи до ~8.6mS, но понякога се появява 'Lader OVERFLOW' и 'Time Overflow'. За толкова вх/изх. трябва data_wait_to_lader_mul_10ms = 2;
- master е готов за използване на обекти;
    - само да се махнат protocol_ELL_defs.h с производните си, putchar и printf
- slave е само за тестове може да се доработи за използване ако го докоментираме и подредим (да се обърне внимание на коментарите с !!!).
# Файлове
## master
- за протокола: dmx_prog.h, master_ELL.c;
- за ладера: lader.h, lader.c
## slave
slave_ELL.c
## други
protocol_ELL_defs.h дефиниции за тестове със сигнал анализатор (за прилагане на обекти да се трие файла и дефинициите).  
# Забележка
Желателно е WiFi да се избягва при този протокол и системи за реално време с реакция под 1 секунда.