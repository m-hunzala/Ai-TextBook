import React from 'react';
import { AuthProvider as BetterAuthProvider } from 'better-auth/react';
import { UserProvider } from './contexts/UserContext';
import config from './config';

// Configuration for Better Auth client - only use in browser
const authConfig = typeof window !== 'undefined'
  ? {
      // The base URL where the auth API is served (adjust if needed)
      baseURL: config.AUTH_API_URL || config.BETTER_AUTH_URL || process.env.AUTH_API_URL || 'http://localhost:4000',
    }
  : undefined;

export default function Root({ children }) {
  // Render without BetterAuthProvider on server-side or if config is unavailable
  if (!authConfig || typeof window === 'undefined') {
    return (
      <UserProvider>
        {children}
      </UserProvider>
    );
  }

  return (
    <BetterAuthProvider config={authConfig}>
      <UserProvider>
        {children}
      </UserProvider>
    </BetterAuthProvider>
  );
}