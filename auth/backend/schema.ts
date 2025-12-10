import { sql } from "drizzle-orm";
import { sqliteTable, text, integer, blob } from "drizzle-orm/sqlite-core";

// Define users table first to avoid circular reference issues
export const users = sqliteTable("user", {
  id: text("id").primaryKey(),
  email: text("email").unique().notNull(),
  emailVerified: integer("email_verified", { mode: "boolean" }).default(false),
  phone: text("phone").unique(),
  phoneVerified: integer("phone_verified", { mode: "boolean" }).default(false),
  firstName: text("first_name"),
  lastName: text("last_name"),
  image: text("image"),
  password: text("password"),
  // TotpSecret is used for two-factor authentication
  totpSecret: blob("totp_secret", { mode: "buffer" }),
  backupCodes: text("backup_codes"),
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Accounts table that references users
export const accounts = sqliteTable("account", {
  id: text("id").primaryKey(),
  userId: text("userId")
    .notNull()
    .references(() => users.id, { onDelete: "cascade" }),
  providerId: text("providerId").notNull(),
  providerAccountId: text("providerAccountId").notNull(),
  refreshToken: text("refreshToken"),
  accessToken: text("accessToken"),
  expiresAt: integer("expiresAt"),
  tokenType: text("tokenType"),
  scope: text("scope"),
  idToken: text("idToken"),
  sessionState: text("sessionState"),
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Sessions table that references users
export const sessions = sqliteTable("session", {
  id: text("id").primaryKey(),
  userId: text("userId")
    .notNull()
    .references(() => users.id, { onDelete: "cascade" }),
  expiresAt: integer("expiresAt").notNull(),
  idleExpiresAt: integer("idle_expires_at").notNull(),
  ipAddress: text("ip_address"),
  userAgent: text("user_agent"),
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Verification table
export const verification = sqliteTable("verification", {
  id: text("id").primaryKey(),
  identifier: text("identifier").notNull(), // Can be email, phone, username, etc
  value: text("value").notNull(), // Verification code or link
  expiresAt: integer("expiresAt").notNull(),
  type: text("type").notNull(), // email-verification, phone-verification, reset-password, etc
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Authenticators table that references users
export const authenticators = sqliteTable("authenticator", {
  id: text("id").primaryKey(),
  userId: text("userId")
    .notNull()
    .references(() => users.id, { onDelete: "cascade" }),
  credentialID: text("credential_id").notNull().unique(),
  credentialPublicKey: blob("credential_public_key", { mode: "buffer" }).notNull(),
  counter: integer("counter").notNull(),
  credentialDeviceType: text("credential_device_type").notNull(),
  credentialBackedUp: integer("credential_backed_up", { mode: "boolean" })
    .notNull()
    .default(false),
  transports: text("transports"),
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Additional table for user metadata specific to this application
export const userMetadata = sqliteTable("user_metadata", {
  id: text("id").primaryKey(),
  userId: text("userId")
    .notNull()
    .references(() => users.id, { onDelete: "cascade" }),
  softwareBackground: text("software_background"),
  hardwareAvailable: text("hardware_available"),
  experienceLevel: text("experience_level"),
  // New fields for Physical AI & Humanoid Robotics book
  programming_experience: text("programming_experience"),
  hardware_experience: text("hardware_experience"),
  gpu_access: text("gpu_access"),
  preferred_language: text("preferred_language"),
  createdAt: integer("created_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp" })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});