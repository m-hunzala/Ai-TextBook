import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def run_migrations():
    """Run database migrations"""
    db_url = os.getenv("NEON_DATABASE_URL")
    if not db_url:
        print("NEON_DATABASE_URL environment variable is required")
        return

    # Connect to the database
    conn = await asyncpg.connect(db_url)

    try:
        # Create migration table if it doesn't exist
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS schema_migrations (
                id SERIAL PRIMARY KEY,
                version VARCHAR(255) UNIQUE NOT NULL,
                applied_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
            )
        """)

        # Read the migration file
        migration_file = os.path.join(os.path.dirname(__file__), "migrations", "001_initial_schema.sql")
        with open(migration_file, "r") as f:
            migration_sql = f.read()

        # Check if migration has already been applied
        migration_version = "001_initial_schema"
        result = await conn.fetchrow(
            "SELECT version FROM schema_migrations WHERE version = $1",
            migration_version
        )

        if result:
            print(f"Migration {migration_version} already applied")
        else:
            # Execute the migration
            await conn.execute(migration_sql)

            # Record that migration was applied
            await conn.execute(
                "INSERT INTO schema_migrations (version) VALUES ($1)",
                migration_version
            )

            print(f"Migration {migration_version} applied successfully")

    finally:
        await conn.close()

if __name__ == "__main__":
    asyncio.run(run_migrations())