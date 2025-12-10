import { drizzle } from "drizzle-orm/better-sqlite3";
import Database from "better-sqlite3";
import * as schema from "./schema.ts";

// Initialize SQLite connection
const sqlite = new Database(process.env.DATABASE_URL || "./auth.db");

export const db = drizzle(sqlite, { schema });