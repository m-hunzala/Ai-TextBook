import { betterAuth } from "better-auth";
import { postgresAdapter } from "better-auth/adapters/postgres";
import { neon } from "@neondatabase/serverless";

const db = neon(process.env.DATABASE_URL || "");

export const auth = betterAuth({
  secret: process.env.BETTER_AUTH_SECRET || "your-super-secret-key-change-in-production",
  trustHost: true,
  database: postgresAdapter(db, {
    // Better Auth tables
  }),
  // Custom fields for user profiles as specified in the requirements
  user: {
    fields: {
      // No additional fields in the users table, using user_profiles table instead
    }
  },
  plugins: [
    // Add additional plugins here if needed
  ],
  socialProviders: {
    // Add social providers if needed
  },
  // Additional configuration options
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true if you want email verification
  },
});