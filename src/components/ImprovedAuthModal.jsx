import React, { useState, useEffect } from 'react';
import { useAuth } from '../auth/authClient';
import { useUserData } from '../contexts/UserContext';

const ImprovedAuthModal = ({ isOpen, onClose }) => {
  const [activeTab, setActiveTab] = useState('login'); // 'login' or 'signup'
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '', // For signup
    programming_experience: '',
    hardware_experience: [],
    gpu_access: '',
    preferred_language: ''
  });
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  const { signIn, signUp } = useAuth();
  const { isAuthenticated } = useUserData();

  // Close modal if user becomes authenticated
  useEffect(() => {
    if (isAuthenticated && isOpen) {
      onClose();
    }
  }, [isAuthenticated, isOpen, onClose]);

  // Handle form input changes
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  };

  // Handle hardware experience selection (multi-select)
  const handleHardwareChange = (option) => {
    setFormData(prev => {
      const hardware = [...prev.hardware_experience];
      const index = hardware.indexOf(option);

      if (index >= 0) {
        hardware.splice(index, 1);
      } else {
        hardware.push(option);
      }

      return { ...prev, hardware_experience: hardware };
    });
  };

  // Validate form
  const validateForm = () => {
    const newErrors = {};

    // Email validation
    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/\S+@\S+\.\S+/.test(formData.email)) {
      newErrors.email = 'Email is invalid';
    }

    // Password validation
    if (!formData.password.trim()) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 6) {
      newErrors.password = 'Password must be at least 6 characters';
    }

    // Name validation (for signup only)
    if (activeTab === 'signup' && !formData.name.trim()) {
      newErrors.name = 'Name is required';
    }

    // Custom fields validation (for signup only)
    if (activeTab === 'signup') {
      if (!formData.programming_experience.trim()) {
        newErrors.programming_experience = 'Programming experience is required';
      }
      if (formData.hardware_experience.length === 0) {
        newErrors.hardware_experience = 'Please select at least one hardware option';
      }
      if (!formData.gpu_access.trim()) {
        newErrors.gpu_access = 'GPU access selection is required';
      }
      if (!formData.preferred_language.trim()) {
        newErrors.preferred_language = 'Preferred language is required';
      }
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      if (activeTab === 'login') {
        // Login
        const result = await signIn({
          email: formData.email,
          password: formData.password
        });

        if (result?.error) {
          setErrors({ general: result.error.message });
        } else {
          onClose(); // Close modal on successful login
        }
      } else {
        // Signup with additional fields
        const result = await signUp({
          email: formData.email,
          password: formData.password,
          name: formData.name,
          programming_experience: formData.programming_experience,
          hardware_experience: formData.hardware_experience,
          gpu_access: formData.gpu_access,
          preferred_language: formData.preferred_language
        });

        if (result?.error) {
          setErrors({ general: result.error.message });
        } else {
          onClose(); // Close modal on successful signup
        }
      }
    } catch (error) {
      console.error(activeTab === 'login' ? 'Login' : 'Signup', 'error:', error);
      setErrors({ general: `Failed to ${activeTab}. Please try again.` });
    } finally {
      setIsLoading(false);
    }
  };

  // Handle social login
  const handleSocialLogin = async (provider) => {
    if (typeof window === 'undefined') {
      console.error('Social login not available in server environment');
      return;
    }

    setIsLoading(true);
    try {
      // Better-Auth social login implementation - redirect to provider
      const authBaseUrl = typeof window !== 'undefined'
        ? window.ENV?.AUTH_API_URL || window.ENV?.BETTER_AUTH_URL || 'http://localhost:4001'
        : 'http://localhost:4001';

      const socialProvider = provider.toLowerCase();
      const callbackURL = encodeURIComponent(window.location.href);

      // Redirect to the appropriate social login provider
      window.location.href = `${authBaseUrl}/api/auth/signin/${socialProvider}?callbackURL=${callbackURL}`;
    } catch (error) {
      console.error(`${provider} login error:`, error);
      setErrors({ general: `Failed to login with ${provider}. Please try again.` });
      setIsLoading(false);
    }
  };

  // Reset form when modal opens or tab changes
  useEffect(() => {
    if (isOpen) {
      setFormData({
        email: '',
        password: '',
        name: '',
        programming_experience: '',
        hardware_experience: [],
        gpu_access: '',
        preferred_language: ''
      });
      setErrors({});
    }
  }, [isOpen, activeTab]);

  if (!isOpen) {
    return null;
  }

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black bg-opacity-55 backdrop-blur-6px animate-fade-in">
      <div className="relative w-full max-w-sm md:max-w-md lg:max-w-lg xl:max-w-xl 2xl:max-w-2xl max-w-[420px] auth-modal-bg rounded-xl shadow-2xl overflow-hidden transform transition-all duration-300 scale-100 animate-slide-up">
        {/* Close Button */}
        <button
          onClick={onClose}
          className="absolute top-4 right-4 auth-modal-close-btn z-10 transition-colors duration-200 hover:scale-110"
          aria-label="Close"
        >
          <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>

        {/* Tabs */}
        <div className="flex border-b border-gray-200 dark:border-gray-700">
          <button
            className={`flex-1 py-4 px-6 text-center font-medium transition-colors duration-200 ${
              activeTab === 'login'
                ? 'auth-modal-tab-active'
                : 'auth-modal-tab-inactive'
            }`}
            onClick={() => setActiveTab('login')}
          >
            Login
          </button>
          <button
            className={`flex-1 py-4 px-6 text-center font-medium transition-colors duration-200 ${
              activeTab === 'signup'
                ? 'auth-modal-tab-active'
                : 'auth-modal-tab-inactive'
            }`}
            onClick={() => setActiveTab('signup')}
          >
            Sign Up
          </button>
        </div>

        {/* Form Content */}
        <div className="p-6 md:p-8">
          {errors.general && (
            <div className="mb-4 p-3 bg-red-50 dark:bg-red-900/20 text-red-700 dark:text-red-300 rounded-lg animate-fade-in">
              {errors.general}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            {activeTab === 'signup' && (
              <div className="mb-4 animate-fade-in">
                <label htmlFor="name" className="block mb-2 text-sm font-medium auth-modal-label">
                  Full Name
                </label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                    errors.name
                      ? 'border-red-500 focus:ring-red-500'
                      : 'auth-modal-input'
                  }`}
                  placeholder="Enter your full name"
                />
                {errors.name && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.name}</p>}
              </div>
            )}

            <div className="mb-4 animate-fade-in">
              <label htmlFor="email" className="block mb-2 text-sm font-medium auth-modal-label">
                Email Address
              </label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleInputChange}
                className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                  errors.email
                    ? 'border-red-500 focus:ring-red-500'
                    : 'auth-modal-input'
                }`}
                placeholder="you@example.com"
              />
              {errors.email && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.email}</p>}
            </div>

            <div className="mb-4 animate-fade-in">
              <label htmlFor="password" className="block mb-2 text-sm font-medium auth-modal-label">
                Password
              </label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                  errors.password
                    ? 'border-red-500 focus:ring-red-500'
                    : 'auth-modal-input'
                }`}
                placeholder="••••••••"
              />
              {errors.password && <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.password}</p>}
            </div>

            {activeTab === 'signup' && (
              <>
                {/* Programming Experience */}
                <div className="mb-4 animate-fade-in">
                  <label htmlFor="programming_experience" className="block mb-2 text-sm font-medium auth-modal-label">
                    Programming Experience
                  </label>
                  <select
                    id="programming_experience"
                    name="programming_experience"
                    value={formData.programming_experience}
                    onChange={handleInputChange}
                    className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                      errors.programming_experience
                        ? 'border-red-500 focus:ring-red-500'
                        : 'auth-modal-input'
                    }`}
                  >
                    <option value="">Select your programming experience</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                  {errors.programming_experience && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.programming_experience}</p>
                  )}
                </div>

                {/* Hardware Experience */}
                <div className="mb-4 animate-fade-in">
                  <label className="block mb-2 text-sm font-medium auth-modal-label">
                    Hardware Experience
                  </label>
                  <div className="grid grid-cols-2 gap-2">
                    {[
                      { value: 'jetson', label: 'Jetson' },
                      { value: 'arduino', label: 'Arduino' },
                      { value: 'raspberry_pi', label: 'Raspberry Pi' },
                      { value: 'none', label: 'None' }
                    ].map((option) => (
                      <div key={option.value} className="flex items-center animate-fade-in">
                        <input
                          type="checkbox"
                          id={`hw-${option.value}`}
                          checked={formData.hardware_experience.includes(option.value)}
                          onChange={() => handleHardwareChange(option.value)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded transition-colors duration-200"
                        />
                        <label htmlFor={`hw-${option.value}`} className="ml-2 text-sm auth-modal-label transition-colors duration-200">
                          {option.label}
                        </label>
                      </div>
                    ))}
                  </div>
                  {errors.hardware_experience && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.hardware_experience}</p>
                  )}
                </div>

                {/* GPU Access */}
                <div className="mb-4 animate-fade-in">
                  <label htmlFor="gpu_access" className="block mb-2 text-sm font-medium auth-modal-label">
                    GPU Access
                  </label>
                  <select
                    id="gpu_access"
                    name="gpu_access"
                    value={formData.gpu_access}
                    onChange={handleInputChange}
                    className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                      errors.gpu_access
                        ? 'border-red-500 focus:ring-red-500'
                        : 'auth-modal-input'
                    }`}
                  >
                    <option value="">Do you have GPU access?</option>
                    <option value="yes">Yes</option>
                    <option value="no">No</option>
                  </select>
                  {errors.gpu_access && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.gpu_access}</p>
                  )}
                </div>

                {/* Preferred Language */}
                <div className="mb-6 animate-fade-in">
                  <label htmlFor="preferred_language" className="block mb-2 text-sm font-medium auth-modal-label">
                    Preferred Language
                  </label>
                  <select
                    id="preferred_language"
                    name="preferred_language"
                    value={formData.preferred_language}
                    onChange={handleInputChange}
                    className={`w-full px-4 py-3 border rounded-lg focus:outline-none focus:ring-2 transition-colors duration-200 ${
                      errors.preferred_language
                        ? 'border-red-500 focus:ring-red-500'
                        : 'auth-modal-input'
                    }`}
                  >
                    <option value="">Select your preferred language</option>
                    <option value="english">English</option>
                    <option value="urdu">Urdu</option>
                  </select>
                  {errors.preferred_language && (
                    <p className="mt-1 text-sm text-red-600 dark:text-red-400 animate-fade-in">{errors.preferred_language}</p>
                  )}
                </div>
              </>
            )}

            <button
              type="submit"
              disabled={isLoading}
              className="w-full py-3 px-4 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-lg transition-all duration-300 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 dark:focus:ring-offset-gray-800 disabled:opacity-50 transform hover:scale-[1.02] active:scale-[0.98]"
            >
              {isLoading ? (
                <span className="flex items-center justify-center">
                  <svg className="animate-spin -ml-1 mr-3 h-5 w-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                  </svg>
                  Processing...
                </span>
              ) : activeTab === 'login' ? (
                'Sign In'
              ) : (
                'Sign Up'
              )}
            </button>

            {/* Social Login Buttons */}
            <div className="mt-6">
              <div className="relative">
                <div className="absolute inset-0 flex items-center">
                  <div className="w-full border-t border-gray-300 dark:border-gray-600"></div>
                </div>
                <div className="relative flex justify-center text-sm">
                  <span className="px-2 bg-gray-50 dark:bg-gray-800 text-gray-500 dark:text-gray-400">
                    Or continue with
                  </span>
                </div>
              </div>

              <div className="mt-6 grid grid-cols-2 gap-3">
                <button
                  type="button"
                  disabled={isLoading}
                  onClick={() => handleSocialLogin('github')}
                  className="w-full inline-flex justify-center py-2 px-4 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm bg-white dark:bg-gray-700 text-sm font-medium text-gray-500 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-gray-600 transition-colors duration-200"
                >
                  <svg className="h-5 w-5" fill="currentColor" viewBox="0 0 20 20">
                    <path fillRule="evenodd" d="M10 0C4.477 0 0 4.484 0 10.017c0 4.425 2.865 8.18 6.839 9.504.5.092.682-.217.682-.483 0-.237-.008-.868-.013-1.703-2.782.605-3.369-1.343-3.369-1.343-.454-1.158-1.11-1.466-1.11-1.466-.908-.62.069-.608.069-.608 1.003.07 1.531 1.032 1.531 1.032.892 1.53 2.341 1.088 2.91.832.092-.647.35-1.088.636-1.338-2.22-.253-4.555-1.113-4.555-4.951 0-1.093.39-1.988 1.029-2.688-.103-.253-.446-1.272.098-2.65 0 0 .84-.27 2.75 1.026A9.564 9.564 0 0110 4.844c.85.004 1.705.115 2.504.337 1.909-1.296 2.747-1.027 2.747-1.027.546 1.379.203 2.398.1 2.651.64.7 1.028 1.595 1.028 2.688 0 3.848-2.339 4.695-4.566 4.942.359.31.678.921.678 1.856 0 1.338-.012 2.419-.012 2.747 0 .268.18.58.688.482A10.019 10.019 0 0020 10.017C20 4.484 15.522 0 10 0z" clipRule="evenodd" />
                  </svg>
                  <span className="ml-2">GitHub</span>
                </button>

                <button
                  type="button"
                  disabled={isLoading}
                  onClick={() => handleSocialLogin('google')}
                  className="w-full inline-flex justify-center py-2 px-4 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm bg-white dark:bg-gray-700 text-sm font-medium text-gray-500 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-gray-600 transition-colors duration-200"
                >
                  <svg className="h-5 w-5" fill="currentColor" viewBox="0 0 24 24">
                    <path d="M12.24 10.285V14.4h6.806c-.275 1.765-2.056 5.174-6.806 5.174-4.095 0-7.439-3.389-7.439-7.574s3.345-7.574 7.439-7.574c2.33 0 3.891.989 4.785 1.849l3.254-3.138C18.189 1.186 15.479 0 12.24 0c-6.635 0-12 5.365-12 12s5.365 12 12 12c6.926 0 11.52-4.869 11.52-11.726 0-.788-.085-1.39-.189-1.989H12.24z"/>
                  </svg>
                  <span className="ml-2">Google</span>
                </button>
              </div>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
};

export default ImprovedAuthModal;