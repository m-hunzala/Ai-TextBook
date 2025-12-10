import React from 'react';
import { useUserData } from '../contexts/UserContext';

const AuthFooter = () => {
  const { isAuthenticated, signOut } = useUserData();

  const handleLogout = async () => {
    try {
      await signOut();
    } catch (error) {
      console.error('Logout error:', error);
    }
  };

  return (
    <div className="footer-auth-section">
      {isAuthenticated ? (
        <button
          onClick={handleLogout}
          className="footer-auth-button"
          style={{
            background: 'none',
            border: 'none',
            color: 'inherit',
            cursor: 'pointer',
            textDecoration: 'underline',
            fontSize: '0.85em',
            padding: 0
          }}
        >
          Logout
        </button>
      ) : (
        <a
          href="/login"
          className="footer-auth-link"
          style={{
            color: 'inherit',
            textDecoration: 'underline',
            fontSize: '0.85em'
          }}
        >
          Login
        </a>
      )}
    </div>
  );
};

export default AuthFooter;