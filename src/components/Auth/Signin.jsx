import React, { useState } from 'react';
import './Auth.css';

const Signin = ({ apiUrl, onSigninComplete }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSignin = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');
    
    try {
      // In a real implementation, this would call Better-Auth signin
      // For now, we'll just simulate the process
      // This would typically interact with the Better-Auth API
      
      // For demo purposes, assume successful signin and call the callback
      setTimeout(() => {
        onSigninComplete && onSigninComplete();
        setLoading(false);
      }, 1000);
    } catch (err) {
      setError(err.message || 'Signin failed');
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Sign In</h2>
        {error && <div className="auth-error">{error}</div>}
        
        <form onSubmit={handleSignin}>
          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>
          
          <button type="submit" disabled={loading} className="auth-button">
            {loading ? 'Signing in...' : 'Sign In'}
          </button>
        </form>
        
        <div className="auth-link">
          Don't have an account? <a href="/signup">Sign up</a>
        </div>
      </div>
    </div>
  );
};

export default Signin;