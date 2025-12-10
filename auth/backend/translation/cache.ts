import fs from 'fs/promises';
import path from 'path';

// Cache configuration
const CACHE_DIR = path.join(process.cwd(), 'cache', 'translations');
const CACHE_FILE_EXTENSION = '.json';
const DEFAULT_CACHE_TTL = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

interface CachedData {
  data: any;
  timestamp: number;
  ttl: number; // time-to-live in milliseconds
}

// Initialize cache directory
async function initCacheDir(): Promise<void> {
  try {
    await fs.access(CACHE_DIR);
  } catch {
    await fs.mkdir(CACHE_DIR, { recursive: true });
  }
}

// Create a valid filename from cache key
function getCacheFilePath(key: string): string {
  // Sanitize key to be a valid filename
  const sanitizedKey = key.replace(/[^a-zA-Z0-9-_]/g, '_');
  return path.join(CACHE_DIR, `${sanitizedKey}${CACHE_FILE_EXTENSION}`);
}

// Get value from cache
export async function getCache(key: string): Promise<any | null> {
  try {
    await initCacheDir();
    const filePath = getCacheFilePath(key);
    
    const fileContent = await fs.readFile(filePath, 'utf8');
    const cachedData: CachedData = JSON.parse(fileContent);
    
    // Check if cache is expired
    const now = Date.now();
    if (now - cachedData.timestamp > cachedData.ttl) {
      // Delete expired cache
      await fs.unlink(filePath);
      return null;
    }
    
    return cachedData.data;
  } catch (error) {
    // Cache miss or error - return null
    return null;
  }
}

// Set value in cache
export async function setCache(key: string, value: any, ttl: number = DEFAULT_CACHE_TTL): Promise<void> {
  try {
    await initCacheDir();
    const filePath = getCacheFilePath(key);
    
    const cacheData: CachedData = {
      data: value,
      timestamp: Date.now(),
      ttl
    };
    
    await fs.writeFile(filePath, JSON.stringify(cacheData, null, 2));
  } catch (error) {
    console.error("Error writing to cache:", error);
    // Fail silently - caching is optional
  }
}

// Delete value from cache
export async function deleteCache(key: string): Promise<void> {
  try {
    const filePath = getCacheFilePath(key);
    await fs.unlink(filePath);
  } catch (error) {
    // Ignore errors - the file may not exist
  }
}

// Clear all cache
export async function clearCache(): Promise<void> {
  try {
    await initCacheDir();
    const files = await fs.readdir(CACHE_DIR);
    
    for (const file of files) {
      if (file.endsWith(CACHE_FILE_EXTENSION)) {
        await fs.unlink(path.join(CACHE_DIR, file));
      }
    }
  } catch (error) {
    console.error("Error clearing cache:", error);
  }
}

// Get cache size/stats (optional)
export async function getCacheStats(): Promise<{ size: number; count: number }> {
  try {
    await initCacheDir();
    const files = await fs.readdir(CACHE_DIR);
    let totalSize = 0;
    
    for (const file of files) {
      if (file.endsWith(CACHE_FILE_EXTENSION)) {
        const filePath = path.join(CACHE_DIR, file);
        const stats = await fs.stat(filePath);
        totalSize += stats.size;
      }
    }
    
    return {
      size: totalSize,
      count: files.filter(f => f.endsWith(CACHE_FILE_EXTENSION)).length
    };
  } catch (error) {
    return { size: 0, count: 0 };
  }
}