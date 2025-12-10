# Better-Auth Integration Guide

## Overview

This document explains how Better-Auth has been integrated into the RAG backend system for enhanced user authentication and profile management.

## Architecture

The integration combines:
- Better-Auth on the frontend for user authentication flows
- JWT-based authentication for API requests
- Neon Postgres for storing extended user profiles
- FastAPI backend endpoints for profile management

## Database Schema

### Better-Auth Tables
The system includes standard Better-Auth tables:
- `users`: Basic user information
- `accounts`: OAuth provider information
- `sessions`: Session management
- `verification_tokens`: Email verification tokens

### Extended User Profile Table
- `user_profiles`: Extended user information collected during signup
  - `user_id`: Foreign key reference to users table
  - `primary_os`: User's primary operating system (Linux/Win/Mac)
  - `gpu_model`: GPU model (RTX model or None)
  - `experience_level`: Experience with ROS/Sim/Isaac (Beginner/Intermediate/Advanced)
  - `preferred_hardware`: Preferred hardware (Jetson/Cloud)

## Frontend Implementation

### Signup Flow
1. User enters basic credentials (email, password)
2. System collects profile information:
   - Primary OS selection
   - GPU model selection
   - Experience level selection
   - Preferred hardware selection
3. Profile is saved to database using `/auth/user-profile` endpoint

### Signin Flow
1. User enters credentials
2. Better-Auth generates JWT
3. JWT is used for subsequent API requests

## Backend Endpoints

### Authentication Endpoints
- `GET /auth/user-info`: Retrieve current user's basic information
- `GET /auth/user-profile`: Retrieve current user's extended profile
- `POST /auth/user-profile`: Update user profile information

### All other endpoints now require Better-Auth authentication:
- `POST /query`
- `POST /answer`
- `POST /answer-stream`
- `POST /log-source-click`
- `POST /analytics`
- `GET /subagents`
- `POST /subagent-call`
- `POST /embed-upsert`

## API Usage

### Getting User Information
```javascript
fetch('/auth/user-info', {
  headers: {
    'Authorization': `Bearer ${jwtToken}`
  }
})
```

### Getting User Profile
```javascript
fetch('/auth/user-profile', {
  headers: {
    'Authorization': `Bearer ${jwtToken}`
  }
})
```

### Updating User Profile
```javascript
fetch('/auth/user-profile', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${jwtToken}`
  },
  body: JSON.stringify({
    profile: {
      primary_os: 'Linux',
      gpu_model: 'RTX 4090',
      experience_level: 'Advanced',
      preferred_hardware: 'Jetson'
    }
  })
})
```

## Setup Instructions

### 1. Environment Configuration
Add the Better-Auth secret to your environment:
```
BETTER_AUTH_SECRET=your-32-character-secret-key-here
```

### 2. Database Migration
Run the migration script to create tables:
```bash
python migrate.py
```

### 3. Frontend Integration
Integrate Better-Auth into your frontend application:
```javascript
import { createAuth, CredentialsProvider } from "better-auth";

export const auth = createAuth({
  secret: process.env.BETTER_AUTH_SECRET,
  trustHost: true,
  plugins: [
    // Add any additional plugins here
  ],
  providers: [
    CredentialsProvider({
      credentials: {
        email: {
          type: "email",
          required: true,
        },
        password: {
          type: "password",
          required: true,
        },
      },
      async authorize(credentials) {
        // Implement your authorization logic here
        // This is a simplified example
        return {
          id: "1",
          email: credentials.email,
          name: "User Name",
        };
      },
    }),
  ],
});
```

### 4. Using the API
All API requests now require a Better-Auth JWT in the Authorization header:
```javascript
const response = await fetch('/answer', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'Authorization': `Bearer ${jwtToken}`
  },
  body: JSON.stringify({
    query: 'Your question here'
  })
});
```

## Security Considerations

1. Use a strong, randomly generated `BETTER_AUTH_SECRET`
2. Ensure HTTPS is used in production
3. Implement rate limiting for authentication endpoints
4. Validate JWT tokens on the backend
5. Store sensitive data appropriately

## User Profile Collection Schema

The signup form collects the following information:

1. **Primary OS**: 
   - Options: Linux, Windows, Mac
   - Purpose: To customize documentation and examples

2. **GPU Model**:
   - Options: Various RTX models or None
   - Purpose: To provide hardware-specific recommendations

3. **Experience Level**:
   - Options: Beginner, Intermediate, Advanced
   - Purpose: To adjust response complexity and guidance

4. **Preferred Hardware**:
   - Options: Jetson, Cloud, Other
   - Purpose: To tailor recommendations and examples

This information is stored in the `user_profiles` table and can be used to customize the user experience.