import React, { useState } from 'react';

function SigninForm({ onSigninSuccess }) {
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    const formData = new FormData(e.target);
    const data = {
      email: formData.get('email'),
      password: formData.get('password')
    };

    try {
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
      });

      const result = await response.json();

      if (response.ok) {
        // Handle successful signin
        if (onSigninSuccess) {
          onSigninSuccess(result);
        }
      } else {
        setError(result.error || 'Sign in failed');
      }
    } catch (err) {
      setError('An error occurred during sign in');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2>Sign In</h2>
      
      {error && <div className="error-message">{error}</div>}
      
      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input 
            type="email" 
            id="email" 
            name="email" 
            required 
            placeholder="Enter your email"
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input 
            type="password" 
            id="password" 
            name="password" 
            required 
            placeholder="Enter your password"
          />
        </div>
        
        <button type="submit" disabled={loading} className="submit-btn">
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>
    </div>
  );
}

export default SigninForm;