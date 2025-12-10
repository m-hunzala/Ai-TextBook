// authAPI.js - Utility functions for authentication API calls

// Signup with custom fields
export const signup = async (userData) => {
  const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4001';
  const response = await fetch(`${authServerUrl}/api/auth/signup`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(userData)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Signup failed');
  }

  return response.json();
};

// Signin
export const signin = async (credentials) => {
  const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4001';
  const response = await fetch(`${authServerUrl}/api/auth/signin`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(credentials)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Sign in failed');
  }

  return response.json();
};

// Signout
export const signout = async () => {
  const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4001';
  const response = await fetch(`${authServerUrl}/api/auth/signout`, {
    method: 'POST'
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Sign out failed');
  }

  return response.json();
};

// Get current user profile
export const getCurrentUserProfile = async () => {
  const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4001';
  const response = await fetch(`${authServerUrl}/api/auth/profile`);

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Unauthorized');
    }
    const error = await response.json();
    throw new Error(error.error || 'Failed to get profile');
  }

  return response.json();
};

// Update user profile
export const updateUserProfile = async (profileData) => {
  const authServerUrl = process.env.AUTH_API_URL || 'http://localhost:4001';
  const response = await fetch(`${authServerUrl}/api/auth/profile`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(profileData)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error || 'Failed to update profile');
  }

  return response.json();
};