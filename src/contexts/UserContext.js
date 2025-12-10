import React, { createContext, useContext, useMemo } from 'react';
import { useAuth } from '../auth/authClient'; // Use our custom auth client

const UserContext = createContext();

export const UserProvider = ({ children }) => {
  const auth = useAuth();

  // Only use auth data if it's available (browser environment)
  const { session, signIn, signOut, isPending } = auth || {
    session: null,
    signIn: () => Promise.reject(new Error('Auth not available')),
    signOut: () => Promise.reject(new Error('Auth not available')),
    isPending: false,
  };

  // Simplified user data structure
  const userData = session ? {
    userId: session.user?.id,
    email: session.user?.email,
    name: session.user?.name,
    // Better Auth uses cookies for session management, not access tokens
  } : null;

  const value = useMemo(() => ({
    userData,
    isAuthenticated: !!session,
    signIn,
    signOut,
    isPending: isPending || false
  }), [userData, session, signIn, signOut, isPending]);

  return (
    <UserContext.Provider value={value}>
      {children}
    </UserContext.Provider>
  );
};

export const useUserData = () => {
  const context = useContext(UserContext);
  if (context === undefined) {
    // Return a safe fallback object instead of throwing an error
    console.warn('useUserData is being used outside of UserProvider - returning defaults');
    return {
      userData: null,
      isAuthenticated: false,
      signIn: () => Promise.reject(new Error('Auth not available')),
      signOut: () => Promise.reject(new Error('Auth not available')),
      isPending: false
    };
  }
  return context;
};