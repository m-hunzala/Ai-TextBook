// pages/api/auth/profile.js (for Next.js) or equivalent serverless function

import { neon } from '@neondatabase/serverless';

const sql = neon(process.env.NEON_DB_URL || '');

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const {
      name,
      email,
      country,
      primaryOS,
      gpu,
      programmingBackground,
      experienceLevel,
      mainDomain,
      hardwareOwned = []
    } = req.body;

    // Validate required fields
    if (!email) {
      return res.status(400).json({ error: 'Email is required' });
    }

    // Insert or update user profile
    const result = await sql(
      `INSERT INTO user_profiles
       (name, email, country, primary_os, gpu, experience_level, main_domain, hardware_owned, programming_background)
       VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
       ON CONFLICT (email)
       DO UPDATE SET
         name = EXCLUDED.name,
         country = EXCLUDED.country,
         primary_os = EXCLUDED.primary_os,
         gpu = EXCLUDED.gpu,
         experience_level = EXCLUDED.experience_level,
         main_domain = EXCLUDED.main_domain,
         hardware_owned = EXCLUDED.hardware_owned,
         programming_background = EXCLUDED.programming_background,
         updated_at = CURRENT_TIMESTAMP
       RETURNING id`,
      [name, email, country, primaryOS, gpu, experienceLevel, mainDomain, hardwareOwned, programmingBackground]
    );

    res.status(200).json({ 
      message: 'Profile saved successfully', 
      profileId: result[0]?.id 
    });
    
  } catch (error) {
    console.error('Error saving profile:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}

export const config = {
  api: {
    bodyParser: true,
  },
};