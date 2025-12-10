import type { Request, Response } from "express";
import { translateMarkdown } from "./translationLogic.ts";
import { getCache, setCache } from "./cache.ts";
import { auth } from "../auth.ts";

export const translateChapter = async (req: Request, res: Response) => {
  try {
    // Get session from request (optional - for caching personalization)
    let session = null;
    try {
      session = await auth.api.getSession({
        headers: req.headers,
        cookies: req.cookies,
      });
    } catch (e) {
      // Session is optional for translation
      console.log("No valid session for translation - that's OK");
    }

    const { chapterId, target } = req.body;

    if (!chapterId || !target) {
      return res.status(400).json({
        error: "chapterId and target language are required"
      });
    }

    // Check cache first
    const cacheKey = `translation:${chapterId}:${target}:${session?.user?.id || 'anonymous'}`;
    const cachedTranslation = await getCache(cacheKey);

    if (cachedTranslation) {
      return res.status(200).json({
        translatedContent: cachedTranslation,
        fromCache: true
      });
    }

    // Get original chapter content (in a real implementation, this would come from your content system)
    // For now, we'll assume it's passed in the request or fetched from a content management system
    const originalContent = req.body.originalContent || null;

    if (!originalContent) {
      return res.status(400).json({
        error: "Original content is required for translation"
      });
    }

    // Perform translation
    let translatedContent;
    try {
      translatedContent = await translateMarkdown(originalContent, target);
    } catch (translationError) {
      console.error("Translation failed:", translationError);
      return res.status(500).json({
        error: "Translation failed. Keeping original content.",
        translatedContent: originalContent, // Return original as fallback
        translationError: translationError.message
      });
    }

    // Cache the result (with error handling)
    try {
      await setCache(cacheKey, translatedContent);
    } catch (cacheError) {
      console.error("Cache error:", cacheError);
      // Continue anyway, as caching is optional
    }

    return res.status(200).json({
      translatedContent,
      fromCache: false
    });
  } catch (error: any) {
    console.error("Error in translateChapter:", error);
    return res.status(500).json({
      error: error.message || "Internal server error during translation"
    });
  }
};