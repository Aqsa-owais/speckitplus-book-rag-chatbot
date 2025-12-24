import React from 'react';
import clsx from 'clsx';
import { useThemeConfig } from '@docusaurus/theme-common';

function Footer() {
  const { footer } = useThemeConfig();

  if (!footer) {
    return null;
  }

  return (
    <footer
      className={clsx('footer', {
        'footer--dark': footer.style === 'dark',
      })}>
      <div className="container container-fluid">
        <div className="footer__inner">
          <div className="footer__copyright" style={{
            textAlign: 'center',
            padding: '2rem 0',
            borderTop: '1px solid var(--ifm-color-emphasis-200)',
            color: 'var(--ifm-footer-color)',
            fontSize: '0.875rem',
          }}>
            Copyright © 2025 Physical AI Robotics Book. Built with Docusaurus.
          </div>
        </div>
      </div>
    </footer>
  );
}

export default Footer;