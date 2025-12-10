import React from 'react';
import Layout from '@theme/Layout';
import UserProfileForm from '@site/src/components/UserProfileForm';

export default function SignupPage() {
  return (
    <Layout title="Sign Up" description="Create your personalized learning profile">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="text--center">Create Your Learning Profile</h1>
            <p className="text--center">
              Sign up to unlock personalized content tailored to your hardware and experience level.
            </p>
            
            <div className="margin-vert--lg">
              <UserProfileForm />
            </div>
            
            <div className="text--center margin-vert--lg">
              <p>Already have an account? <a href="/signin">Sign in here</a></p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}