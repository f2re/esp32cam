#ifndef CHUNKED_UPLOADER_H
#define CHUNKED_UPLOADER_H

#include <Arduino.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "esp_task_wdt.h"

/**
 * ChunkedUploader - класс для загрузки изображений с ESP32-CAM на сервер
 * с поддержкой resume и chunked-передачи
 * 
 * Особенности:
 * - Разбиение на чанки по 8 КБ для стабильности Wi-Fi
 * - Автоматический resume после разрыва соединения
 * - Retry-механизм с exponential backoff
 * - Логирование использования памяти в debug-режиме
 * 
 * Автор: f2re
 * Дата: 22.01.2026
 */

#define CHUNK_SIZE 8192        // 8 КБ — оптимально для ESP32 Wi-Fi buffer
#define MAX_RETRIES 3          // максимум попыток на чанк
#define RETRY_DELAY_MS 1000    // начальная задержка между попытками
#define CONNECT_TIMEOUT_MS 10000  // таймаут подключения к серверу

// Режим отладки памяти
#ifndef DEBUG_MEMORY
#define DEBUG_MEMORY 1         // 1 = включить логирование памяти
#endif

class ChunkedUploader {
private:
    String serverHost;
    uint16_t serverPort;
    String uploadId;
    size_t totalSize;
    size_t bytesUploaded;
    
    // Логирование памяти
    void logMemoryStatus(const char* context);
    
    // Retry-механизм для одного чанка
    bool uploadChunkWithRetry(const uint8_t* data, size_t len, size_t offset);
    
    // Запрос статуса для resume
    bool getUploadStatus(size_t& resumeOffset);
    
    // Парсинг JSON-ответов (простой метод без библиотеки)
    bool parseJsonString(const String& json, const char* key, String& value);
    bool parseJsonInt(const String& json, const char* key, int& value);
    bool parseJsonBool(const String& json, const char* key, bool& value);
    
public:
    ChunkedUploader(const String& host, uint16_t port);
    
    /**
     * Инициализация upload-сессии на сервере
     * @param camId ID камеры
     * @param fileSize размер файла в байтах
     * @param fileHash MD5-хеш для проверки целостности (опционально)
     * @return true если успешно
     */
    bool initUpload(const String& camId, size_t fileSize, const String& fileHash = "");
    
    /**
     * Загрузка framebuffer с автоматическим resume
     * @param fb указатель на camera framebuffer
     * @return true если успешно
     */
    bool uploadFramebuffer(camera_fb_t* fb);
    
    /**
     * Финализация upload (проверка целостности на сервере)
     * @return true если успешно
     */
    bool finalize();
    
    /**
     * Получение прогресса загрузки в процентах
     * @return процент завершения (0.0-100.0)
     */
    float getProgress() const;
    
    /**
     * Получение upload_id (для сохранения в RTC-память при reboot)
     * @return upload_id строка
     */
    String getUploadId() const { return uploadId; }
    
    /**
     * Установка upload_id (для resume после reboot)
     */
    void setUploadId(const String& id) { uploadId = id; }
};

/**
 * Вспомогательные функции для логирования памяти
 */
namespace MemoryLogger {
    void printMemoryInfo(const char* tag = nullptr);
    size_t getFreeHeap();
    size_t getFreePsram();
    size_t getLargestFreeBlock();
}

#endif // CHUNKED_UPLOADER_H