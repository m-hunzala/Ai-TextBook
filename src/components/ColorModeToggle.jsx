import React from 'react';
import clsx from 'clsx';
import { useColorMode, useThemeConfig } from '@docusaurus/theme-common';
import styles from './ColorModeToggle.module.css';

const DEFAULT_TOGGLE_COLOR_DARK = 'var(--ifm-color-emphasis-200)';
const DEFAULT_TOGGLE_COLOR_LIGHT = 'var(--ifm-color-emphasis-700)';

function ColorModeToggle({ className, buttonClassName, value, onChange }) {
  const { colorMode, setColorMode } = useColorMode();
  const {
    colorMode: themeColorMode = {}
  } = useThemeConfig();

  const {
    darkIcon = '🌙',
    darkIconStyle,
    lightIcon = '☀️',
    lightIconStyle,
    syncToggle = false
  } = themeColorMode.switchConfig || {};

  const icons = {
    light: (
      <span
        style={{
          verticalAlign: 'middle',
          lineHeight: '0',
          fontSize: '1rem',
          ...lightIconStyle,
        }}>
        {lightIcon}
      </span>
    ),
    dark: (
      <span
        style={{
          verticalAlign: 'middle',
          lineHeight: '0',
          fontSize: '1rem',
          ...darkIconStyle,
        }}>
        {darkIcon}
      </span>
    ),
  };

  const currentColor = colorMode === 'dark' ? DEFAULT_TOGGLE_COLOR_DARK : DEFAULT_TOGGLE_COLOR_LIGHT;

  return (
    <div className={clsx(styles.colorModeToggle, className)}>
      <button
        className={clsx(
          'clean-btn',
          styles.toggleButton,
          buttonClassName,
        )}
        type="button"
        onClick={() => setColorMode(colorMode === 'dark' ? 'light' : 'dark')}
        title={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
        aria-label={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
        style={{ color: currentColor }}>
        {icons[colorMode]}
      </button>
    </div>
  );
}

export default React.memo(ColorModeToggle);