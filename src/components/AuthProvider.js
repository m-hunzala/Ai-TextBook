import React from 'react';
import { AuthProvider as BetterAuthProvider } from 'better-auth/react';

const authConfig = {
  fetchOptions: {
    // Use the correct API endpoint for Better Auth
    baseURL: '/api/auth', // This should match where Better Auth is mounted
  }
};

export default function AuthProvider({ children }) {
  return (
    <BetterAuthProvider config={authConfig}>
      {children}
    </BetterAuthProvider>
  );
}