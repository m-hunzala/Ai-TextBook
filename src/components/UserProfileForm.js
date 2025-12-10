import { useState } from 'react';
import useClientAuth from '../hooks/useClientAuth';

const UserProfileForm = () => {
  const { client: authClient, session, isReady } = useClientAuth();
  const signUp = authClient?.signUp;
  const signIn = authClient?.signIn;
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    country: '',
    primaryOS: '',
    gpu: '',
    programmingBackground: '',
    aiExperienceLevel: 'Beginner',
    experienceLevel: 'Beginner',
    mainDomain: 'Robotics',
    hardwareOwned: []
  });
  
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  
  const osOptions = [
    'Windows', 'macOS', 'Linux', 'Web Browser', 'Cloud'
  ];
  
  const gpuOptions = [
    'RTX 4090', 'RTX 4080', 'RTX 4070 Ti', 'RTX 4070', 'RTX 4060 Ti', 
    'RTX 3090', 'RTX 3080', 'RTX 3070', 'RTX 3060 Ti', 'RTX 3060', 
    'RTX 2080 Ti', 'RTX 2080', 'RTX 2070', 'RTX 2060', 'GTX 1080 Ti',
    'Other (specify below)'
  ];
  
  const hardwareOptions = [
    'Jetson', 'RealSense', 'Unitree', 'NVIDIA Jetson AGX', 
    'NVIDIA Jetson Orin', 'Intel Realsense D400', 'Intel Realsense L500',
    'Other'
  ];

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    
    if (type === 'checkbox') {
      setFormData(prev => {
        const currentHardware = prev.hardwareOwned || [];
        if (checked) {
          return {
            ...prev,
            [name]: [...currentHardware, value]
          };
        } else {
          return {
            ...prev,
            [name]: currentHardware.filter(item => item !== value)
          };
        }
      });
    } else {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      // First, create the user account with Better Auth
      if (!signUp) {
        setError('Authentication client not ready. Please try again.');
        setIsLoading(false);
        return;
      }

      const authResult = await signUp({
        email: formData.email,
        password: formData.password,
        name: formData.name
      });

      if (authResult?.error) {
        setError(authResult.error.message);
        setIsLoading(false);
        return;
      }

      // Then save the profile to Neon Postgres
      const profileResponse = await fetch('/api/auth/profile', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          name: formData.name,
          email: formData.email,
          country: formData.country,
          primaryOS: formData.primaryOS,
          gpu: formData.gpu,
          programmingBackground: formData.programmingBackground,
          experienceLevel: formData.aiExperienceLevel, // Use AI experience level
          mainDomain: formData.mainDomain,
          hardwareOwned: formData.hardwareOwned,
          userId: authResult.data?.user?.id
        })
      });

      if (!profileResponse.ok) {
        const errorData = await profileResponse.json();
        throw new Error(errorData.message || 'Failed to save profile');
      }

      // Redirect or show success message
      alert('Profile created successfully!');
      window.location.href = '/signin'; // Redirect to sign-in after successful profile creation
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="max-w-2xl mx-auto p-6 bg-white rounded-lg shadow-md">
      <h2 className="text-2xl font-bold mb-6 text-gray-800">Create Your Profile</h2>
      
      {error && (
        <div className="mb-4 p-3 bg-red-100 text-red-700 rounded-md">
          {error}
        </div>
      )}
      
      <form onSubmit={handleSubmit} className="space-y-4">
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label htmlFor="name" className="block text-sm font-medium text-gray-700 mb-1">
              Full Name *
            </label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="John Doe"
            />
          </div>
          
          <div>
            <label htmlFor="email" className="block text-sm font-medium text-gray-700 mb-1">
              Email Address *
            </label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="john@example.com"
            />
          </div>
        </div>
        
        <div>
          <label htmlFor="password" className="block text-sm font-medium text-gray-700 mb-1">
            Password *
          </label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            minLength={8}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            placeholder="At least 8 characters"
          />
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label htmlFor="country" className="block text-sm font-medium text-gray-700 mb-1">
              Country
            </label>
            <select
              id="country"
              name="country"
              value={formData.country}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="">Select Country</option>
              <option value="United States">United States</option>
              <option value="United Kingdom">United Kingdom</option>
              <option value="Canada">Canada</option>
              <option value="Germany">Germany</option>
              <option value="France">France</option>
              <option value="Japan">Japan</option>
              <option value="China">China</option>
              <option value="India">India</option>
              <option value="Brazil">Brazil</option>
              <option value="Other">Other</option>
            </select>
          </div>

          <div>
            <label htmlFor="primaryOS" className="block text-sm font-medium text-gray-700 mb-1">
              Primary Operating System
            </label>
            <select
              id="primaryOS"
              name="primaryOS"
              value={formData.primaryOS}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="">Select OS</option>
              {osOptions.map(os => (
                <option key={os} value={os}>{os}</option>
              ))}
            </select>
          </div>
        </div>

        <div>
          <label htmlFor="programmingBackground" className="block text-sm font-medium text-gray-700 mb-1">
            Programming Background
          </label>
          <select
            id="programmingBackground"
            name="programmingBackground"
            value={formData.programmingBackground}
            onChange={handleChange}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="">Select your programming background</option>
            <option value="None">None</option>
            <option value="Beginner">Beginner (Basic understanding)</option>
            <option value="Intermediate">Intermediate (Some experience)</option>
            <option value="Advanced">Advanced (Extensive experience)</option>
            <option value="Professional">Professional (Working in industry)</option>
          </select>
        </div>
        
        <div>
          <label htmlFor="gpu" className="block text-sm font-medium text-gray-700 mb-1">
            GPU (Graphics Processing Unit)
          </label>
          <select
            id="gpu"
            name="gpu"
            value={formData.gpu}
            onChange={handleChange}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="">Select GPU</option>
            {gpuOptions.map(gpu => (
                <option key={gpu} value={gpu}>{gpu}</option>
            ))}
          </select>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label htmlFor="aiExperienceLevel" className="block text-sm font-medium text-gray-700 mb-1">
              AI Experience Level
            </label>
            <select
              id="aiExperienceLevel"
              name="aiExperienceLevel"
              value={formData.aiExperienceLevel}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          <div>
            <label htmlFor="mainDomain" className="block text-sm font-medium text-gray-700 mb-1">
              Main Domain of Interest
            </label>
            <select
              id="mainDomain"
              name="mainDomain"
              value={formData.mainDomain}
              onChange={handleChange}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value="Robotics">Robotics</option>
              <option value="Computer Vision">Computer Vision</option>
              <option value="Embedded">Embedded Systems</option>
            </select>
          </div>
        </div>
        
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Hardware Owned
          </label>
          <div className="grid grid-cols-2 md:grid-cols-3 gap-2">
            {hardwareOptions.map(hw => (
              <div key={hw} className="flex items-center">
                <input
                  type="checkbox"
                  id={`hardware-${hw}`}
                  name="hardwareOwned"
                  value={hw}
                  checked={formData.hardwareOwned?.includes(hw)}
                  onChange={handleChange}
                  className="h-4 w-4 text-blue-600 border-gray-300 rounded focus:ring-blue-500"
                />
                <label htmlFor={`hardware-${hw}`} className="ml-2 text-sm text-gray-700">
                  {hw}
                </label>
              </div>
            ))}
          </div>
        </div>
        
        <div className="pt-4">
          <button
            type="submit"
            disabled={isLoading}
            className="w-full bg-blue-600 text-white py-2 px-4 rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 disabled:opacity-50 disabled:cursor-not-allowed"
          >
            {isLoading ? 'Creating Profile...' : 'Create Profile'}
          </button>
        </div>
      </form>
    </div>
  );
};

export default UserProfileForm;