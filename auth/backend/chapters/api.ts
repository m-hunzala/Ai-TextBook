import type { Request, Response } from "express";
import { auth } from "../auth.ts";
import { db } from "../database.ts";
import { userMetadata } from "../schema.ts";
import { eq } from "drizzle-orm";
import { applyPersonalizationRules } from "./personalization.ts";

export const personalizeChapter = async (req: Request, res: Response) => {
  try {
    // Get session from request
    const session = await auth.api.getSession({
      headers: req.headers,
      cookies: req.cookies,
    });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    // Fetch user metadata from our custom table
    const metadata = await db
      .select()
      .from(userMetadata)
      .where(eq(userMetadata.userId, session.user.id))
      .limit(1);

    if (metadata.length === 0) {
      return res.status(404).json({ error: "User metadata not found" });
    }

    const { id } = req.params; // chapter ID
    const { markdown } = req.body; // original chapter content

    // Apply personalization rules based on user metadata
    const personalizedMarkdown = applyPersonalizationRules(
      markdown, 
      metadata[0]
    );

    return res.status(200).json({ 
      personalizedContent: personalizedMarkdown 
    });
  } catch (error: any) {
    console.error("Error personalizing chapter:", error);
    return res.status(500).json({ error: error.message || "Internal server error" });
  }
};