#!/usr/bin/env python3
"""
Setup script for Neon Postgres database.
Run this script to create all necessary tables.
"""

import os
import sys
from pathlib import Path

try:
    import psycopg2
    from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT
except ImportError:
    print("Error: psycopg2 not installed. Install it with: pip install psycopg2-binary")
    sys.exit(1)


def setup_database(connection_string: str):
    """Create database tables from schema.sql"""
    schema_path = Path(__file__).parent / "schema.sql"
    
    if not schema_path.exists():
        print(f"Error: schema.sql not found at {schema_path}")
        sys.exit(1)
    
    try:
        # Connect to database
        conn = psycopg2.connect(connection_string)
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
        cursor = conn.cursor()
        
        # Read and execute schema
        with open(schema_path, 'r') as f:
            schema_sql = f.read()
        
        cursor.execute(schema_sql)
        print("✓ Database schema created successfully")
        
        cursor.close()
        conn.close()
        
    except psycopg2.Error as e:
        print(f"Error setting up database: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # Get connection string from environment or command line
    connection_string = os.getenv("NEON_DATABASE_URL")
    
    if not connection_string:
        if len(sys.argv) > 1:
            connection_string = sys.argv[1]
        else:
            print("Usage: python setup_db.py <connection_string>")
            print("Or set NEON_DATABASE_URL environment variable")
            sys.exit(1)
    
    setup_database(connection_string)

