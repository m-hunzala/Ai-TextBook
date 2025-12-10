import React, { useState } from 'react';
import './Auth.css';

const Signup = ({ apiUrl, onSignupComplete }) => {
  const [step, setStep] = useState(1); // 1: credentials, 2: profile
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  
  // Profile state
  const [primaryOS, setPrimaryOS] = useState('');
  const [gpuModel, setGpuModel] = useState('');
  const [experienceLevel, setExperienceLevel] = useState('');
  const [preferredHardware, setPreferredHardware] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSignup = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');
    
    // Step 1: Simulate Better-Auth signup (in real implementation, this would call Better-Auth)
    // For now, we'll just move to profile collection
    try {
      // Simulate API call to signup endpoint (Better-Auth handles this)
      // This is a simplified version - in real integration, Better-Auth would handle the signup
      setStep(2);
    } catch (err) {
      setError(err.message || 'Signup failed');
    } finally {
      setLoading(false);
    }
  };

  const handleProfileSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');
    
    try {
      // Send profile data to our backend
      const response = await fetch(`${apiUrl}/auth/user-profile`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: 'temp_user_id', // This would come from Better-Auth session
          profile: {
            primary_os: primaryOS,
            gpu_model: gpuModel,
            experience_level: experienceLevel,
            preferred_hardware: preferredHardware
          }
        })
      });
      
      if (!response.ok) {
        throw new Error('Failed to save profile');
      }
      
      onSignupComplete && onSignupComplete();
    } catch (err) {
      setError(err.message || 'Failed to save profile');
    } finally {
      setLoading(false);
    }
  };

  const osOptions = ['Linux', 'Windows', 'Mac'];
  const gpuOptions = ['RTX 2060', 'RTX 2070', 'RTX 2080', 'RTX 3060', 'RTX 3070', 'RTX 3080', 'RTX 3090', 'RTX 4060', 'RTX 4070', 'RTX 4080', 'RTX 4090', 'None'];
  const experienceOptions = ['Beginner', 'Intermediate', 'Advanced'];
  const hardwareOptions = ['Jetson', 'Cloud', 'Other'];

  return (
    <div className="auth-container">
      {step === 1 ? (
        <div className="auth-form">
          <h2>Create Account</h2>
          {error && <div className="auth-error">{error}</div>}
          
          <form onSubmit={handleSignup}>
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                required
              />
            </div>
            
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
              {loading ? 'Creating...' : 'Continue'}
            </button>
          </form>
        </div>
      ) : (
        <div className="auth-form">
          <h2>Tell Us About Yourself</h2>
          {error && <div className="auth-error">{error}</div>}
          
          <form onSubmit={handleProfileSubmit}>
            <div className="form-group">
              <label>Primary OS</label>
              <div className="radio-group">
                {osOptions.map(option => (
                  <label key={option} className="radio-option">
                    <input
                      type="radio"
                      name="primaryOS"
                      value={option}
                      checked={primaryOS === option}
                      onChange={() => setPrimaryOS(option)}
                      required
                    />
                    <span>{option}</span>
                  </label>
                ))}
              </div>
            </div>
            
            <div className="form-group">
              <label>GPU Model (or None)</label>
              <select 
                value={gpuModel} 
                onChange={(e) => setGpuModel(e.target.value)}
                required
              >
                <option value="">Select GPU</option>
                {gpuOptions.map(option => (
                  <option key={option} value={option}>{option}</option>
                ))}
              </select>
            </div>
            
            <div className="form-group">
              <label>Experience Level with ROS/Sim/Isaac</label>
              <div className="radio-group">
                {experienceOptions.map(option => (
                  <label key={option} className="radio-option">
                    <input
                      type="radio"
                      name="experienceLevel"
                      value={option}
                      checked={experienceLevel === option}
                      onChange={() => setExperienceLevel(option)}
                      required
                    />
                    <span>{option}</span>
                  </label>
                ))}
              </div>
            </div>
            
            <div className="form-group">
              <label>Preferred Hardware</label>
              <div className="radio-group">
                {hardwareOptions.map(option => (
                  <label key={option} className="radio-option">
                    <input
                      type="radio"
                      name="preferredHardware"
                      value={option}
                      checked={preferredHardware === option}
                      onChange={() => setPreferredHardware(option)}
                      required
                    />
                    <span>{option}</span>
                  </label>
                ))}
              </div>
            </div>
            
            <button type="submit" disabled={loading} className="auth-button">
              {loading ? 'Saving...' : 'Complete Signup'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default Signup;