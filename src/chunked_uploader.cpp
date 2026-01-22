#include "chunked_uploader.h"
#include <MD5Builder.h>

// =============================================================================
// Memory Logger Implementation
// =============================================================================

namespace MemoryLogger {
    size_t getFreeHeap() {
        return ESP.getFreeHeap();
    }
    
    size_t getFreePsram() {
        return ESP.getFreePsram();
    }
    
    size_t getLargestFreeBlock() {
        return ESP.getMaxAllocHeap();
    }
    
    void printMemoryInfo(const char* tag) {
#if DEBUG_MEMORY
        const char* prefix = tag ? tag : "[MEM]";
        
        size_t freeHeap = getFreeHeap();
        size_t freePsram = getFreePsram();
        size_t largestBlock = getLargestFreeBlock();
        size_t totalHeap = ESP.getHeapSize();
        size_t totalPsram = ESP.getPsramSize();
        
        Serial.printf("%s Heap: %u/%u KB (largest block: %u KB)\n",
                     prefix,
                     freeHeap / 1024, totalHeap / 1024,
                     largestBlock / 1024);
        
        Serial.printf("%s PSRAM: %u/%u KB\n",
                     prefix,
                     freePsram / 1024, totalPsram / 1024);
        
        // Процент использования
        float heapUsage = totalHeap > 0 ? 100.0f * (1.0f - (float)freeHeap / totalHeap) : 0.0f;
        float psramUsage = totalPsram > 0 ? 100.0f * (1.0f - (float)freePsram / totalPsram) : 0.0f;
        
        Serial.printf("%s Usage: Heap %.1f%%, PSRAM %.1f%%\n",
                     prefix, heapUsage, psramUsage);
#endif
    }
}

// =============================================================================
// ChunkedUploader Implementation
// =============================================================================

ChunkedUploader::ChunkedUploader(const String& host, uint16_t port)
    : serverHost(host), serverPort(port), totalSize(0), bytesUploaded(0) {
}

void ChunkedUploader::logMemoryStatus(const char* context) {
#if DEBUG_MEMORY
    char tag[64];
    snprintf(tag, sizeof(tag), "[UPLOAD:%s]", context);
    MemoryLogger::printMemoryInfo(tag);
#endif
}

bool ChunkedUploader::parseJsonString(const String& json, const char* key, String& value) {
    String pattern = String("\"") + key + "\":\"";
    int idx = json.indexOf(pattern);
    if (idx < 0) return false;
    
    int start = idx + pattern.length();
    int end = json.indexOf('"', start);
    if (end < 0) return false;
    
    value = json.substring(start, end);
    return true;
}

bool ChunkedUploader::parseJsonInt(const String& json, const char* key, int& value) {
    String pattern = String("\"") + key + "\":";
    int idx = json.indexOf(pattern);
    if (idx < 0) return false;
    
    int colon = json.indexOf(':', idx);
    if (colon < 0) return false;
    colon++;
    
    while (colon < (int)json.length() && (json[colon] == ' ' || json[colon] == '"')) colon++;
    
    String num = "";
    while (colon < (int)json.length() && (isdigit((unsigned char)json[colon]) || json[colon] == '-')) {
        num += json[colon++];
    }
    
    if (num.length() == 0) return false;
    value = num.toInt();
    return true;
}

bool ChunkedUploader::parseJsonBool(const String& json, const char* key, bool& value) {
    String pattern = String("\"") + key + "\":";
    int idx = json.indexOf(pattern);
    if (idx < 0) return false;
    
    int colon = json.indexOf(':', idx);
    if (colon < 0) return false;
    colon++;
    
    while (colon < (int)json.length() && json[colon] == ' ') colon++;
    
    if (json.substring(colon).startsWith("true")) {
        value = true;
        return true;
    }
    if (json.substring(colon).startsWith("false")) {
        value = false;
        return true;
    }
    
    return false;
}

bool ChunkedUploader::initUpload(const String& camId, size_t fileSize, const String& fileHash) {
    logMemoryStatus("INIT_START");
    
    HTTPClient http;
    WiFiClient client;
    
    String url = "http://" + serverHost + ":" + String(serverPort) + "/upload/init";
    
    http.begin(client, url);
    http.setTimeout(CONNECT_TIMEOUT_MS);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    String postData = "cam_id=" + camId + "&file_size=" + String(fileSize);
    
    if (fileHash.length() > 0) {
        postData += "&file_hash=" + fileHash;
    }
    
    Serial.printf("[UPLOAD] Init: cam_id=%s, size=%u bytes\n", camId.c_str(), fileSize);
    
    int httpCode = http.POST(postData);
    
    if (httpCode == 200) {
        String response = http.getString();
        
        String id;
        if (!parseJsonString(response, "upload_id", id)) {
            Serial.println("[UPLOAD] Failed to parse upload_id");
            http.end();
            return false;
        }
        
        uploadId = id;
        totalSize = fileSize;
        bytesUploaded = 0;
        
        Serial.printf("[UPLOAD] Initialized: upload_id=%s\n", uploadId.c_str());
        logMemoryStatus("INIT_SUCCESS");
        http.end();
        return true;
    }
    
    Serial.printf("[UPLOAD] Init failed: HTTP %d\n", httpCode);
    if (httpCode > 0) {
        Serial.printf("[UPLOAD] Response: %s\n", http.getString().c_str());
    }
    http.end();
    return false;
}

bool ChunkedUploader::getUploadStatus(size_t& resumeOffset) {
    HTTPClient http;
    WiFiClient client;
    
    String url = "http://" + serverHost + ":" + String(serverPort) + 
                 "/upload/status/" + uploadId;
    
    http.begin(client, url);
    http.setTimeout(5000);
    int httpCode = http.GET();
    
    if (httpCode == 200) {
        String response = http.getString();
        
        int bytesReceived;
        if (!parseJsonInt(response, "bytes_received", bytesReceived)) {
            http.end();
            return false;
        }
        
        resumeOffset = (size_t)bytesReceived;
        http.end();
        return true;
    }
    
    http.end();
    return false;
}

bool ChunkedUploader::uploadChunkWithRetry(const uint8_t* data, size_t len, size_t offset) {
    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        esp_task_wdt_reset();
        
        HTTPClient http;
        WiFiClient client;
        
        String url = "http://" + serverHost + ":" + String(serverPort) + 
                     "/upload/chunk/" + uploadId + "?offset=" + String(offset);
        
        http.begin(client, url);
        http.setTimeout(CONNECT_TIMEOUT_MS);
        http.addHeader("Content-Type", "application/octet-stream");
        
        int httpCode = http.sendRequest("PUT", (uint8_t*)data, len);
        
        if (httpCode == 200 || httpCode == 201) {
            String response = http.getString();
            
            int serverBytesReceived;
            if (parseJsonInt(response, "bytes_received", serverBytesReceived)) {
                bytesUploaded = (size_t)serverBytesReceived;
            } else {
                bytesUploaded = offset + len; // fallback
            }
            
            http.end();
            return true;
        }
        
        if (httpCode == 409) {
            // Conflict — неправильный offset, нужен resume
            Serial.println("[UPLOAD] Offset conflict, requesting resume offset...");
            
            size_t resumeOffset;
            if (getUploadStatus(resumeOffset)) {
                bytesUploaded = resumeOffset;
                Serial.printf("[UPLOAD] Resume from offset %u\n", resumeOffset);
            }
            
            http.end();
            return false; // caller должен пересчитать offset
        }
        
        Serial.printf("[UPLOAD] Chunk attempt %d/%d failed: HTTP %d\n", 
                     attempt + 1, MAX_RETRIES, httpCode);
        
        if (httpCode > 0) {
            Serial.printf("[UPLOAD] Response: %s\n", http.getString().c_str());
        }
        
        http.end();
        
        if (attempt < MAX_RETRIES - 1) {
            delay(RETRY_DELAY_MS * (attempt + 1)); // exponential backoff
        }
    }
    
    return false;
}

bool ChunkedUploader::uploadFramebuffer(camera_fb_t* fb) {
    if (!fb || fb->len == 0) {
        Serial.println("[UPLOAD] Invalid framebuffer");
        return false;
    }
    
    logMemoryStatus("BEFORE_UPLOAD");
    
    const uint8_t* data = fb->buf;
    size_t remaining = fb->len;
    int chunkIndex = 0;
    
    Serial.printf("[UPLOAD] Starting upload: %u bytes in %u chunks\n",
                 totalSize, (totalSize + CHUNK_SIZE - 1) / CHUNK_SIZE);
    
    while (bytesUploaded < totalSize) {
        esp_task_wdt_reset();
        
        size_t offset = bytesUploaded;
        size_t chunkLen = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        // Указатель на текущий чанк
        const uint8_t* chunkData = data + offset;
        
        Serial.printf("[UPLOAD] Chunk %d: offset=%u, len=%u\n", 
                     chunkIndex, offset, chunkLen);
        
        // Отправка чанка с retry
        if (!uploadChunkWithRetry(chunkData, chunkLen, offset)) {
            // Если произошёл conflict (409), пересчитываем offset
            if (bytesUploaded < offset) {
                Serial.printf("[UPLOAD] Resuming from %u instead of %u\n", 
                             bytesUploaded, offset);
                remaining = totalSize - bytesUploaded;
                continue; // повтор с новым offset
            }
            
            Serial.println("[UPLOAD] Chunk upload failed after retries");
            return false;
        }
        
        remaining -= chunkLen;
        chunkIndex++;
        
        // Логирование прогресса
        Serial.printf("[UPLOAD] Progress: %u/%u bytes (%.1f%%)\n", 
                     bytesUploaded, totalSize, getProgress());
    }
    
    logMemoryStatus("AFTER_UPLOAD");
    Serial.println("[UPLOAD] All chunks uploaded successfully");
    return true;
}

bool ChunkedUploader::finalize() {
    HTTPClient http;
    WiFiClient client;
    
    String url = "http://" + serverHost + ":" + String(serverPort) + 
                 "/upload/finalize/" + uploadId;
    
    Serial.println("[UPLOAD] Finalizing...");
    
    http.begin(client, url);
    http.setTimeout(10000);
    int httpCode = http.POST("");
    
    bool success = (httpCode == 200);
    
    if (success) {
        String response = http.getString();
        Serial.println("[UPLOAD] Finalized successfully");
        Serial.printf("[UPLOAD] Response: %s\n", response.c_str());
    } else {
        Serial.printf("[UPLOAD] Finalize failed: HTTP %d\n", httpCode);
        if (httpCode > 0) {
            Serial.printf("[UPLOAD] Response: %s\n", http.getString().c_str());
        }
    }
    
    http.end();
    logMemoryStatus("FINALIZE");
    return success;
}

float ChunkedUploader::getProgress() const {
    return (totalSize > 0) ? (100.0f * bytesUploaded / totalSize) : 0.0f;
}