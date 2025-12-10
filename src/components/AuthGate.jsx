import React, { useState, useEffect } from 'react';
import { useUserData } from '../contexts/UserContext';
import AuthModal from '../components/AuthModal';

// Separate component that uses the hook - this should be rendered only when
// wrapped by UserProvider
const AuthGateContent = ({ children }) => {
  const { isAuthenticated, isPending } = useUserData();
  const [showAuthModal, setShowAuthModal] = useState(false);

  // Show auth modal if user is not authenticated and data has loaded
  useEffect(() => {
    if (!isPending && !isAuthenticated) {
      // Show modal after a short delay to let the page load
      const timer = setTimeout(() => {
        setShowAuthModal(true);
      }, 1000); // 1 second delay to avoid modal appearing immediately

      return () => clearTimeout(timer);
    }
  }, [isPending, isAuthenticated]);

  // If user is authenticated or still loading, show the children
  if (isAuthenticated || isPending) {
    return children;
  }

  // If user is not authenticated, show the modal
  return (
    <>
      {children}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
      />
    </>
  );
};

const AuthGate = ({ children }) => {
  // Check if we're in the browser environment
  const isBrowser = typeof window !== 'undefined';

  // Only render the content that uses hooks when in browser
  if (!isBrowser) {
    return <>{children}</>;
  }

  // In browser, render the main content component that uses hooks
  return <AuthGateContent>{children}</AuthGateContent>;
};

export default AuthGate;