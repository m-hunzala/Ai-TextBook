import React, { useState } from 'react';
import useClientAuth from '@site/src/hooks/useClientAuth';
import Layout from '@theme/Layout';

export default function SigninPage() {
  const { client: authClient, session, isReady } = useClientAuth();
  const signIn = authClient?.signIn;
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    if (!signIn) {
      setError('Authentication client not ready. Please try again.');
      setIsLoading(false);
      return;
    }

    try {
      const result = await signIn({
        email: formData.email,
        password: formData.password
      });

      if (result?.error) {
        setError(result.error.message);
      } else {
        // Redirect to home or dashboard after successful sign-in
        if (typeof window !== 'undefined') {
          window.location.href = '/';
        }
      }
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  return (
    <Layout title="Sign In" description="Sign in to access your personalized learning experience">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="text--center">Sign In to Your Account</h1>
            <p className="text--center">
              Access your personalized learning experience.
            </p>

            {error && (
              <div className="alert alert--danger margin-bottom--md">
                {error}
              </div>
            )}

            <div className="margin-vert--lg">
              <div className="card">
                <div className="card__body">
                  <form onSubmit={handleSubmit}>
                    <div className="margin-bottom--lg">
                      <label htmlFor="email">Email Address</label>
                      <input
                        type="email"
                        id="email"
                        name="email"
                        value={formData.email}
                        onChange={handleChange}
                        placeholder="your@email.com"
                        className="form-control"
                        required
                      />
                    </div>
                    <div className="margin-bottom--lg">
                      <label htmlFor="password">Password</label>
                      <input
                        type="password"
                        id="password"
                        name="password"
                        value={formData.password}
                        onChange={handleChange}
                        placeholder="••••••••"
                        className="form-control"
                        required
                      />
                    </div>
                    <div className="margin-bottom--lg">
                      <button
                        type="submit"
                        className="button button--primary button--block"
                        disabled={isLoading}
                      >
                        {isLoading ? 'Signing In...' : 'Sign In'}
                      </button>
                    </div>
                  </form>

                  <div className="text--center">
                    <a href="/signup">Don't have an account? Sign up</a>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}