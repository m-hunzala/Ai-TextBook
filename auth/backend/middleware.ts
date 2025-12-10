import type { Request, Response, NextFunction } from "express";
import { auth } from "./auth.ts";

export const requireAuth = async (req: Request, res: Response, next: NextFunction) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers,
      cookies: req.cookies,
    });

    if (!session) {
      return res.status(401).json({ error: "Authentication required" });
    }

    // Add user info to request object for use in downstream handlers
    (req as any).user = session.user;
    next();
  } catch (error) {
    console.error("Authentication middleware error:", error);
    return res.status(500).json({ error: "Authentication error" });
  }
};