import React, { useState } from 'react';

function SignOut({ onSignOut }) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const handleSignOut = async () => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/signout', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Include auth token if required by your implementation
          // 'Authorization': `Bearer ${localStorage.getItem('auth-token')}`
        }
      });

      if (response.ok) {
        if (onSignOut) {
          onSignOut();
        }
      } else {
        const result = await response.json();
        setError(result.error || 'Sign out failed');
      }
    } catch (err) {
      setError('An error occurred during sign out');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <button 
      onClick={handleSignOut} 
      disabled={loading}
      className="signout-btn"
    >
      {loading ? 'Signing Out...' : 'Sign Out'}
    </button>
  );
}

export default SignOut;