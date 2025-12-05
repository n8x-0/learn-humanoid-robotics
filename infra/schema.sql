-- Neon Postgres Database Schema for RAG Chatbot
-- Run this script to set up the database tables

-- Documents table: stores metadata about textbook chapters/sections
CREATE TABLE IF NOT EXISTS documents (
    doc_id VARCHAR(255) PRIMARY KEY,
    title VARCHAR(500) NOT NULL,
    url VARCHAR(1000) NOT NULL,
    content TEXT,
    section_id VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chunks table: stores document chunks with metadata
CREATE TABLE IF NOT EXISTS chunks (
    chunk_id VARCHAR(255) PRIMARY KEY,
    doc_id VARCHAR(255) NOT NULL REFERENCES documents(doc_id) ON DELETE CASCADE,
    section_id VARCHAR(255) NOT NULL,
    text_content TEXT NOT NULL,
    chunk_index INTEGER NOT NULL,
    url VARCHAR(1000) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create index for faster lookups
CREATE INDEX IF NOT EXISTS idx_chunks_doc_id ON chunks(doc_id);
CREATE INDEX IF NOT EXISTS idx_chunks_section_id ON chunks(section_id);

-- Users table (optional for bonus features)
CREATE TABLE IF NOT EXISTS users (
    user_id VARCHAR(255) PRIMARY KEY,
    email VARCHAR(255) UNIQUE,
    profile JSONB, -- Stores software/hardware background
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chat logs table: records of chatbot interactions
CREATE TABLE IF NOT EXISTS chat_logs (
    log_id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) REFERENCES users(user_id) ON DELETE SET NULL,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    mode VARCHAR(50) NOT NULL, -- 'full_corpus' or 'selection'
    citations JSONB, -- Array of citation objects
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create index for chat logs
CREATE INDEX IF NOT EXISTS idx_chat_logs_user_id ON chat_logs(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_logs_created_at ON chat_logs(created_at);

-- Personalization cache table (optional for bonus features)
CREATE TABLE IF NOT EXISTS personalization_cache (
    cache_id SERIAL PRIMARY KEY,
    user_id VARCHAR(255) NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    section_id VARCHAR(255) NOT NULL,
    personalized_content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, section_id)
);

-- Translations cache table (optional for bonus features)
CREATE TABLE IF NOT EXISTS translations_cache (
    translation_id SERIAL PRIMARY KEY,
    section_id VARCHAR(255) NOT NULL,
    locale VARCHAR(10) NOT NULL, -- e.g., 'ur' for Urdu
    translated_text TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(section_id, locale)
);

-- Create indexes for cache tables
CREATE INDEX IF NOT EXISTS idx_personalization_user_section ON personalization_cache(user_id, section_id);
CREATE INDEX IF NOT EXISTS idx_translations_section_locale ON translations_cache(section_id, locale);

