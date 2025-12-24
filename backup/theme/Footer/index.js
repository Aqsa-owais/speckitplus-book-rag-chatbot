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
        <div className="footer__copyright" style={{
          textAlign: 'center',
          color: '#334155',
          fontSize: '0.875rem',
          opacity: 0.8
        }}>
          {/* Add only the required copyright text here */}
        </div>
      </div>
    </footer>
  );
}

export default Footer;