mod payload {
    use super::super::{MotorStop, Payload};

    #[test]
    fn parse_standalone() {
        let (remainder, res) = Payload::parse("A").unwrap();
        assert!(remainder.len() == 0);
        assert_eq!(res, Payload::StartMotor);
    }

    #[test]
    fn parse_subparser() {
        let (remainder, res) = Payload::parse("S1").unwrap();
        assert!(remainder.len() == 0);
        assert_eq!(res, Payload::StopMotor(MotorStop::BrakeRamp));
    }

    #[test]
    fn parse_read_subparser() {
        let (remainder, res) = Payload::parse("Zu133742").unwrap();
        assert!(remainder.len() == 0);
        assert_eq!(res, Payload::MinFrequency(Some(133742)));
    }

    #[test]
    #[should_panic]
    fn parse_invalid_key() {
        let (_, _) = Payload::parse("some_key_that_doesn't_exist133742").unwrap();
    }

    #[test]
    #[should_panic]
    fn parse_read_invalid_key() {
        let (_, _) = Payload::parse("Zsome_key_that_doesn't_exist133742").unwrap();
    }

    #[test]
    #[should_panic]
    fn parse_missing_value() {
        let (_, _) = Payload::parse("S").unwrap();
    }

    #[test]
    #[should_panic]
    fn parse_read_missing_value() {
        let (_, _) = Payload::parse("Zo").unwrap();
    }

    #[test]
    fn parse_rotation_direction_change() {
        let (remainder, res) = Payload::parse("t1").unwrap();
        assert!(remainder.len() == 0);
        assert_eq!(res, Payload::RotationDirectionChange(Some(true)));
    }

    #[test]
    fn fmt_standalone() {
        let pl = Payload::StartMotor;
        assert_eq!(format!("{}", pl), "A");
    }

    #[test]
    fn fmt_with_value() {
        let pl = Payload::StopMotor(MotorStop::BrakeRamp);
        assert_eq!(format!("{}", pl), "S1");
    }

    #[test]
    fn fmt_write() {
        let pl = Payload::TravelDistance(Some(-1337));
        assert_eq!(format!("{}", pl), "s-1337");
    }

    #[test]
    fn fmt_read() {
        let pl = Payload::MinFrequency(None);
        assert_eq!(format!("{}", pl), "Zu");
    }
}

mod cmd {
    use super::super::{Cmd, Payload};

    #[test]
    fn fmt_single() {
        let cmd = Cmd {
            address: Some(42),
            payload: Payload::StartMotor,
        };
        assert_eq!(format!("{}", cmd), "#42A\r");
    }

    #[test]
    fn fmt_all() {
        let cmd = Cmd {
            address: None,
            payload: Payload::StartMotor,
        };
        assert_eq!(format!("{}", cmd), "#*A\r")
    }
}
