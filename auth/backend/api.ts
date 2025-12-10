import { auth } from "./auth.ts";
import { db } from "./database.ts";
import { userMetadata } from "./schema.ts";
import { eq } from "drizzle-orm";
import type { Request, Response } from "express";

// Sign up with custom fields
export const signUp = async (req: Request, res: Response) => {
  try {
    const {
      email,
      password,
      name,
      programming_experience,
      hardware_experience,
      gpu_access,
      preferred_language
    } = req.body;

    // First, create the user with Better Auth
    const authUser = await auth.api.internal.signUp({
      body: {
        email,
        password,
        name,
      },
      userAgent: req.get('User-Agent') || "",
      ip: (req.ip as string) || "",
    });

    if (!authUser) {
      return res.status(400).json({ error: "Sign up failed" });
    }

    // Then, create user metadata in our custom table with new fields
    const metadata = await db
      .insert(userMetadata)
      .values({
        userId: authUser.id,
        programming_experience,
        hardware_experience: hardware_experience || [],
        gpu_access,
        preferred_language,
        // Keep old fields for backward compatibility
        softwareBackground: req.body.softwareBackground || programming_experience,
        hardwareAvailable: req.body.hardwareAvailable || hardware_experience || [],
        experienceLevel: req.body.experienceLevel || programming_experience,
      })
      .returning();

    // Return user data with metadata
    return res.status(200).json({
      user: {
        id: authUser.id,
        email: authUser.email,
        name: authUser.name,
      },
      metadata: metadata[0],
    });
  } catch (error: any) {
    console.error("Error during sign up:", error);
    return res.status(500).json({ error: error.message || "Sign up failed" });
  }
};

// Sign in
export const signIn = async (req: Request, res: Response) => {
  try {
    const { email, password } = req.body;

    // Sign in with Better Auth
    const session = await auth.api.signInWithPassword({
      body: {
        email,
        password,
      },
      userAgent: req.get('User-Agent') || "",
      ip: (req.ip as string) || "",
    });

    if (!session) {
      return res.status(400).json({ error: "Invalid credentials" });
    }

    // Set session cookie
    res.setHeader('Set-Cookie', session.sessionCookie);

    return res.status(200).json({ 
      user: session.user,
      session: session.session 
    });
  } catch (error: any) {
    console.error("Error during sign in:", error);
    return res.status(500).json({ error: error.message || "Sign in failed" });
  }
};

// Sign out
export const signOut = async (req: Request, res: Response) => {
  try {
    // Get session token from either header or cookie
    const token = req.headers.authorization?.split(' ')[1] || 
                  req.cookies?.[auth.sessionConfig.cookieName] || 
                  req.get('Cookie')?.split(';')
                    .find(c => c.trim().startsWith(`${auth.sessionConfig.cookieName}=`))
                    ?.split('=')[1];

    if (!token) {
      return res.status(401).json({ error: "No session found" });
    }

    // Sign out with Better Auth
    await auth.api.signOut({
      sessionToken: token,
    });

    // Clear session cookie
    res.clearCookie(auth.sessionConfig.cookieName);

    return res.status(200).json({ message: "Signed out successfully" });
  } catch (error: any) {
    console.error("Error during sign out:", error);
    return res.status(500).json({ error: error.message || "Sign out failed" });
  }
};

// Get current user's profile (secure endpoint)
export const getCurrentUserProfile = async (req: Request, res: Response) => {
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

    // Return user profile with metadata
    return res.status(200).json({
      user: {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name,
        ...metadata[0], // Include custom metadata fields
      },
    });
  } catch (error: any) {
    console.error("Error getting user profile:", error);
    return res.status(500).json({ error: error.message || "Internal server error" });
  }
};

// Update user's profile (secure endpoint)
export const updateUserProfile = async (req: Request, res: Response) => {
  try {
    // Get session from request
    const session = await auth.api.getSession({
      headers: req.headers,
      cookies: req.cookies,
    });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    const {
      softwareBackground,
      hardwareAvailable,
      experienceLevel,
      programming_experience,
      hardware_experience,
      gpu_access,
      preferred_language
    } = req.body;

    // Check if user metadata exists
    const existingMetadata = await db
      .select()
      .from(userMetadata)
      .where(eq(userMetadata.userId, session.user.id))
      .limit(1);

    let updatedMetadata;
    if (existingMetadata.length > 0) {
      // Update existing metadata
      updatedMetadata = await db
        .update(userMetadata)
        .set({
          softwareBackground,
          hardwareAvailable: hardwareAvailable || [],
          experienceLevel,
          programming_experience,
          hardware_experience: hardware_experience || [],
          gpu_access,
          preferred_language,
          updatedAt: new Date(),
        })
        .where(eq(userMetadata.userId, session.user.id))
        .returning();
    } else {
      // Create new metadata record
      updatedMetadata = await db
        .insert(userMetadata)
        .values({
          userId: session.user.id,
          softwareBackground,
          hardwareAvailable: hardwareAvailable || [],
          experienceLevel,
          programming_experience,
          hardware_experience: hardware_experience || [],
          gpu_access,
          preferred_language,
        })
        .returning();
    }

    return res.status(200).json({ metadata: updatedMetadata[0] });
  } catch (error: any) {
    console.error("Error updating user profile:", error);
    return res.status(500).json({ error: error.message || "Internal server error" });
  }
};

// Delete user's profile (secure endpoint)
export const deleteUserProfile = async (req: Request, res: Response) => {
  try {
    // Get session from request
    const session = await auth.api.getSession({
      headers: req.headers,
      cookies: req.cookies,
    });

    if (!session) {
      return res.status(401).json({ error: "Unauthorized" });
    }

    // Delete user metadata
    await db
      .delete(userMetadata)
      .where(eq(userMetadata.userId, session.user.id));

    // Note: This doesn't delete the Better Auth user record itself
    // To delete the user completely, you'd need to call auth.api.deleteUser()

    return res.status(200).json({ message: "Profile metadata deleted successfully" });
  } catch (error: any) {
    console.error("Error deleting user profile:", error);
    return res.status(500).json({ error: error.message || "Internal server error" });
  }
};