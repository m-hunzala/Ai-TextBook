import React, { useState, useEffect } from 'react';
import { useUserData } from '../contexts/UserContext';
import ImprovedAuthModal from '../components/ImprovedAuthModal';

// Separate component that uses the hook - this should be rendered only when
// wrapped by UserProvider
const AuthManagerContent = () => {
  const { isAuthenticated, signOut, isPending } = useUserData();
  const [isModalOpen, setIsModalOpen] = useState(false);

  // Handle authentication state changes
  useEffect(() => {
    // If user becomes authenticated while modal is open, close it
    if (isAuthenticated && isModalOpen) {
      setIsModalOpen(false);
    }
  }, [isAuthenticated, isModalOpen]);

  const handleLoginClick = () => {
    setIsModalOpen(true);
  };

  const handleLogout = async () => {
    try {
      await signOut();
    } catch (error) {
      console.error('Logout error:', error);
    }
  };

  return (
    <div className="navbar__auth-manager">
      {isAuthenticated ? (
        <button
          onClick={handleLogout}
          className="navbar__auth-button text-sm font-medium text-white bg-red-600 hover:bg-red-700 focus:outline-none focus:ring-2 focus:ring-red-500 focus:ring-offset-2 dark:focus:ring-offset-gray-800"
        >
          Logout
        </button>
      ) : (
        <button
          onClick={handleLoginClick}
          className="navbar__auth-button text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 dark:focus:ring-offset-gray-800"
        >
          Login
        </button>
      )}

      <ImprovedAuthModal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
      />
    </div>
  );
};

const AuthManager = () => {
  // Check if we're in the browser environment
  const isBrowser = typeof window !== 'undefined';

  // Only render the content that uses hooks when in browser
  if (!isBrowser) {
    return (
      <div className="navbar__auth-manager">
        <button
          className="navbar__auth-button text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 dark:focus:ring-offset-gray-800"
          disabled
        >
          Loading...
        </button>
      </div>
    );
  }

  // In browser, render the main content component that uses hooks
  return <AuthManagerContent />;
};

export default AuthManager;