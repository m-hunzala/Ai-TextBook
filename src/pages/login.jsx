import React, { useState, useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import clsx from 'clsx';
import Layout from '@theme/Layout';

export default function Login() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const location = useLocation();
  const history = useHistory();

  // Check if user is already logged in
  useEffect(() => {
    const isAuthenticated = localStorage.getItem('auth') === 'true';
    if (isAuthenticated) {
      history.push('/');
    }
  }, [history]);

  const handleLogin = (e) => {
    e.preventDefault();

    // Simple validation
    if (!username || !password) {
      setError('Please enter both username and password');
      return;
    }

    // For demo purposes, accept any non-empty credentials
    // In a real app, you would validate against a backend
    if (username && password) {
      localStorage.setItem('auth', 'true');
      localStorage.setItem('currentUser', username);
      history.push('/');
    } else {
      setError('Invalid credentials');
    }
  };

  return (
    <Layout title="Login" description="Login to your account">
      <div className={clsx('hero hero--primary', 'login-page')}>
        <div className="container">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <div className="card">
                <div className="card__header">
                  <h2>Login to Your Account</h2>
                </div>
                <div className="card__body">
                  {error && (
                    <div className="alert alert--danger" role="alert">
                      {error}
                    </div>
                  )}
                  <form onSubmit={handleLogin}>
                    <div className="form-group">
                      <label htmlFor="username">Username</label>
                      <input
                        type="text"
                        id="username"
                        className="form-control"
                        value={username}
                        onChange={(e) => setUsername(e.target.value)}
                        placeholder="Enter your username"
                      />
                    </div>
                    <div className="form-group">
                      <label htmlFor="password">Password</label>
                      <input
                        type="password"
                        id="password"
                        className="form-control"
                        value={password}
                        onChange={(e) => setPassword(e.target.value)}
                        placeholder="Enter your password"
                      />
                    </div>
                    <div className="form-group">
                      <button type="submit" className="button button--primary button--block">
                        Login
                      </button>
                    </div>
                  </form>
                </div>
                <div className="card__footer">
                  <p>Don't have an account? <a href="/register">Register here</a></p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <style jsx>{`
        .login-page {
          min-height: 100vh;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .card {
          width: 100%;
          margin-top: 1rem;
        }

        .form-group {
          margin-bottom: 1rem;
        }

        .form-group label {
          display: block;
          margin-bottom: 0.5rem;
          font-weight: 500;
        }

        .form-control {
          width: 100%;
          padding: 0.75rem;
          border: 1px solid var(--ifm-color-emphasis-300);
          border-radius: var(--ifm-button-border-radius);
          background-color: var(--ifm-background-surface-color);
          color: var(--ifm-text-color);
          font-size: 1rem;
        }

        .form-control:focus {
          outline: none;
          border-color: var(--ifm-color-primary);
          box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.2);
        }

        .button--block {
          width: 100%;
        }
      `}</style>
    </Layout>
  );
}