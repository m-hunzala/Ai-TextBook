import React, { useState } from 'react';

function SignupForm({ onSignupSuccess }) {
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    const formData = new FormData(e.target);
    const data = {
      email: formData.get('email'),
      password: formData.get('password'),
      name: formData.get('name'),  // Better-Auth requires a name
      softwareBackground: formData.get('software'),
      hardwareAvailable: Array.from(e.target.querySelectorAll('input[name=hardware]:checked')).map(cb => cb.value),
      experienceLevel: formData.get('experience')
    };

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
      });

      const result = await response.json();

      if (response.ok) {
        // Handle successful signup
        if (onSignupSuccess) {
          onSignupSuccess(result);
        }
      } else {
        setError(result.error || 'Signup failed');
      }
    } catch (err) {
      setError('An error occurred during signup');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Account</h2>
      
      {error && <div className="error-message">{error}</div>}
      
      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="name">Name *</label>
          <input 
            type="text" 
            id="name" 
            name="name" 
            required 
            placeholder="Enter your name"
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="email">Email *</label>
          <input 
            type="email" 
            id="email" 
            name="email" 
            required 
            placeholder="Enter your email"
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="password">Password *</label>
          <input 
            type="password" 
            id="password" 
            name="password" 
            required 
            placeholder="Create a password"
          />
        </div>
        
        <div className="form-group">
          <label htmlFor="software">Software Background *</label>
          <textarea 
            id="software" 
            name="software" 
            required 
            placeholder="Describe your software background, programming experience, and relevant skills..."
            rows="4"
          />
        </div>
        
        <div className="form-group">
          <label>Hardware Available *</label>
          <div className="checkbox-group">
            <label className="checkbox-option">
              <input 
                type="checkbox" 
                name="hardware" 
                value="rtx_40x" 
              />
              RTX 40x Series
            </label>
            <label className="checkbox-option">
              <input 
                type="checkbox" 
                name="hardware" 
                value="rtx_30x" 
              />
              RTX 30x Series
            </label>
            <label className="checkbox-option">
              <input 
                type="checkbox" 
                name="hardware" 
                value="jetson_orin" 
              />
              Jetson Orin
            </label>
            <label className="checkbox-option">
              <input 
                type="checkbox" 
                name="hardware" 
                value="none" 
              />
              None
            </label>
          </div>
        </div>
        
        <div className="form-group">
          <label htmlFor="experience">Experience Level *</label>
          <select 
            id="experience" 
            name="experience" 
            required
          >
            <option value="">Select your experience level</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>
        
        <button type="submit" disabled={loading} className="submit-btn">
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
}

export default SignupForm;