// src/auth/authClient.js
// Initialize better-auth client for frontend
import { createAuthClient } from "better-auth/react";
import { signup } from "../../auth/frontend/authAPI";  // Import the signup API function

// Initialize the auth client only in browser environment
let _useAuth;
if (typeof window !== 'undefined') {
  const authBaseUrl = window.ENV?.AUTH_API_URL || window.ENV?.BETTER_AUTH_URL || 'http://localhost:4001';
  const { useAuth: authHook } = createAuthClient({
    baseURL: authBaseUrl,
  });
  _useAuth = authHook;
} else {
  // Fallback for server-side rendering/building
  _useAuth = () => ({
    session: null,
    signIn: () => Promise.reject(new Error('Auth not available on server')),
    signOut: () => Promise.reject(new Error('Auth not available on server')),
    isPending: false,
  });
}

// Custom hook that extends the default useAuth with signup functionality
export const useAuth = () => {
  if (typeof window === 'undefined') {
    // Return a mock auth object during SSR/build
    return {
      session: null,
      signIn: () => Promise.reject(new Error('Auth not available on server')),
      signOut: () => Promise.reject(new Error('Auth not available on server')),
      isPending: false,
      signUp: () => Promise.reject(new Error('Auth not available on server'))
    };
  }

  const auth = _useAuth();

  // Add signup function to the auth object
  const customSignup = async (userData) => {
    try {
      const result = await signup(userData);
      // The signup API handles creating user in both Better Auth and our database
      return result;
    } catch (error) {
      return { error: { message: error.message } };
    }
  };

  return {
    ...auth,
    signUp: customSignup
  };
};