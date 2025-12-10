import React, { useState, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  softwareBackground?: string;
  hardwareAvailable?: string[];
  experienceLevel?: string;
}

const AuthComponent: React.FC = () => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [formType, setFormType] = useState<'signin' | 'signup'>('signin');
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareAvailable: [] as string[],
    experienceLevel: 'beginner',
  });
  const [error, setError] = useState<string | null>(null);

  // Check if user is already logged in
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        const response = await fetch('/api/auth/profile', {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('auth-token')}`,
          },
        });
        
        if (response.ok) {
          const data = await response.json();
          setUser(data.user);
        }
      } catch (err) {
        console.error('Error checking auth status:', err);
      } finally {
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    
    if (name === 'hardwareAvailable') {
      // Handle checkbox array
      const checked = (e.target as HTMLInputElement).checked;
      const option = e.target.value;
      
      setFormData(prev => ({
        ...prev,
        hardwareAvailable: checked
          ? [...prev.hardwareAvailable, option]
          : prev.hardwareAvailable.filter(item => item !== option)
      }));
    } else {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    try {
      // Get form data using FormData API like in the example
      const form = e.target as HTMLFormElement;
      const formData = new FormData(form);

      let response;
      if (formType === 'signup') {
        // Collect form data similar to the example
        const data = {
          email: formData.get('email') as string,
          password: formData.get('password') as string,
          name: formData.get('name') as string,
          softwareBackground: formData.get('softwareBackground') as string,
          hardwareAvailable: Array.from(form.querySelectorAll('input[name="hardwareAvailable"]:checked'))
            .map((el: any) => el.value),
          experienceLevel: formData.get('experienceLevel') as string,
        };

        response = await fetch('/api/auth/signup', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(data),
        });
      } else {
        const data = {
          email: formData.get('email') as string,
          password: formData.get('password') as string,
        };

        response = await fetch('/api/auth/signin', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(data),
        });
      }

      if (response.ok) {
        const data = await response.json();

        // Note: Better-Auth typically handles session via cookies,
        // not necessarily through localStorage tokens
        if (data.session) {
          // Session is typically handled by cookies in Better-Auth
        }

        // Fetch user profile after successful auth
        const profileResponse = await fetch('/api/auth/profile');

        if (profileResponse.ok) {
          const profileData = await profileResponse.json();
          setUser(profileData.user);
        }
      } else {
        const errorData = await response.json();
        setError(errorData.error || 'Authentication failed');
      }
    } catch (err) {
      setError('An error occurred during authentication');
      console.error(err);
    }
  };

  const handleSignOut = async () => {
    try {
      await fetch('/api/auth/signout', {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('auth-token')}`,
        },
      });
      
      localStorage.removeItem('auth-token');
      setUser(null);
    } catch (err) {
      console.error('Error signing out:', err);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="auth-container">
      {user ? (
        <div className="user-profile">
          <h2>Welcome, {user.name}!</h2>
          <p>Email: {user.email}</p>
          {user.softwareBackground && <p>Software Background: {user.softwareBackground}</p>}
          {user.hardwareAvailable && user.hardwareAvailable.length > 0 && (
            <p>Hardware Available: {user.hardwareAvailable.join(', ')}</p>
          )}
          {user.experienceLevel && <p>Experience Level: {user.experienceLevel}</p>}
          <button onClick={handleSignOut}>Sign Out</button>
        </div>
      ) : (
        <div className="auth-form">
          <div className="form-toggle">
            <button 
              className={formType === 'signin' ? 'active' : ''}
              onClick={() => setFormType('signin')}
            >
              Sign In
            </button>
            <button 
              className={formType === 'signup' ? 'active' : ''}
              onClick={() => setFormType('signup')}
            >
              Sign Up
            </button>
          </div>

          <form onSubmit={handleSubmit}>
            {formType === 'signup' && (
              <div className="form-group">
                <label htmlFor="name">Name</label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  required
                />
              </div>
            )}

            <div className="form-group">
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleInputChange}
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                required
              />
            </div>

            {formType === 'signup' && (
              <>
                <div className="form-group">
                  <label htmlFor="softwareBackground">Software Background</label>
                  <textarea
                    id="softwareBackground"
                    name="softwareBackground"
                    value={formData.softwareBackground}
                    onChange={handleInputChange}
                    required
                  />
                </div>

                <div className="form-group">
                  <label>Hardware Available</label>
                  <div className="checkbox-group">
                    {['rtx_40x', 'rtx_30x', 'jetson_orin', 'none'].map(option => (
                      <label key={option} className="checkbox-option">
                        <input
                          type="checkbox"
                          name="hardwareAvailable"
                          value={option}
                          checked={formData.hardwareAvailable.includes(option)}
                          onChange={handleInputChange}
                        />
                        {option.replace('_', ' ').toUpperCase()}
                      </label>
                    ))}
                  </div>
                </div>

                <div className="form-group">
                  <label htmlFor="experienceLevel">Experience Level</label>
                  <select
                    id="experienceLevel"
                    name="experienceLevel"
                    value={formData.experienceLevel}
                    onChange={handleInputChange}
                    required
                  >
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>
              </>
            )}

            {error && <div className="error">{error}</div>}

            <button type="submit">
              {formType === 'signup' ? 'Sign Up' : 'Sign In'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default AuthComponent;